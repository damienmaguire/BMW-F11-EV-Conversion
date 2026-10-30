#!/bin/sh
# Host unit checks for the FIXED ts-capture build (pass-through repeater):
# TS-CAPTURE record encoder / timestamp matching. (The MCP251863 checks live on
# the ts-capture branch; this branch has no V2 CAN driver.)
set -e
cd "$(dirname "$0")"
if [ -f ../../src/mcp251863.c ]; then
gcc -std=c11 -Wall -Wextra -Werror -Istubs -I../../src -o /tmp/pico_flexray_test_host \
    test_host.c ../../src/mcp251863.c ../../src/mcp251863_bittiming.c
/tmp/pico_flexray_test_host
fi
gcc -std=c11 -Wall -Wextra -Werror -I../../src -o /tmp/pico_flexray_test_ts_record test_ts_record.c
/tmp/pico_flexray_test_ts_record "${1:-/tmp/pico_flexray_ts_sample_stream.bin}"
# DDE emulator core (src/dde_emu_core.c): CRCs, E2E, table rebuild, PIO packing,
# schedule maths, lock FSM. The committed 2 s vector set must rebuild bit-exactly;
# if the run 5 capture is present, every DDE frame of the run is checked as well.
# v1c: dynamic 0xE6/0xE8/0xFB builder vs every real DDE frame of run 5 (dde_dyn_run5.txt),
# DTS/packing/tracker unit checks, and (capture present) the tracker vs the whole run 5 bus.
gcc -std=c11 -Wall -Wextra -Werror -I../../src -o /tmp/pico_flexray_test_dde_emu \
    test_dde_emu_core.c ../../src/dde_emu_core.c -lm
RUN5_BIN="${DDE_RUN5_BIN:-/workspace/f11-ts-run5/f11_run5.csv.bin}"
RUN5_CSV="${DDE_RUN5_CSV:-/workspace/f11-ts-run5/f11_run5.csv}"
if [ -f "$RUN5_BIN" ]; then
    python3 ../../tools/dde_emu/gen_dde_tables.py --bin "$RUN5_BIN" --out /tmp/pico_flexray_dde_tables_check.h \
        --vectors /tmp/pico_flexray_dde_vectors_all.txt > /dev/null
    cmp /tmp/pico_flexray_dde_tables_check.h ../../src/dde_emu_tables.h
    echo "dde_emu_tables.h: regenerated from the capture, identical"
    DYNTRACK=""
    if [ -f "$RUN5_CSV" ]; then
        python3 ../../tools/dde_emu/extract_dyn.py "$RUN5_CSV" /tmp/pico_flexray_dyn_run5_all.txt
        DYNTRACK=dyntrack:/tmp/pico_flexray_dyn_run5_all.txt
    fi
    /tmp/pico_flexray_test_dde_emu strict:dde_vectors_run5_2s.txt /tmp/pico_flexray_dde_vectors_all.txt \
        dyn:dde_dyn_run5.txt $DYNTRACK
else
    echo "(run 5 capture not found: full-run vector check skipped)"
    /tmp/pico_flexray_test_dde_emu strict:dde_vectors_run5_2s.txt dyn:dde_dyn_run5.txt
fi
# DDE emulator firmware (src/dde_emu.c, the real file) replayed against car test 1's
# captured FR1 timing (emu_run1_fr1_1600ms.txt, from f11_emu1.csv) with a model of
# TIMER0, the PIO TX/fire SMs and the transceiver echo. See test_dde_emu_replay.c.
# Two builds: the release build (v1c, 0xE6/0xE8/0xFB on) and all dynamic frames off
# (= v1b behaviour + pre-ign statics) - the v1b scenarios must pass on both.
gcc -std=c11 -Wall -Wextra -Werror -Iemu_stubs -I../../src -o /tmp/pico_flexray_test_dde_emu_replay \
    test_dde_emu_replay.c ../../src/dde_emu.c ../../src/dde_emu_core.c -lm
gcc -std=c11 -Wall -Wextra -Werror -Iemu_stubs -I../../src -DEMU_DYN_E6=0 -DEMU_DYN_FB=0 -DEMU_DYN_E8=0 \
    -o /tmp/pico_flexray_test_dde_emu_replay_nodyn test_dde_emu_replay.c ../../src/dde_emu.c ../../src/dde_emu_core.c -lm
for b in "" _nodyn; do
for sc in run1 run1-spike run1-bigspike lateecho foreign stuck noecho; do
    /tmp/pico_flexray_test_dde_emu_replay$b emu_run1_fr1_1600ms.txt $sc > /tmp/pico_flexray_replay_$sc$b.txt ||
        { cat /tmp/pico_flexray_replay_$sc$b.txt; exit 1; }
    echo "$(tail -n 1 /tmp/pico_flexray_replay_$sc$b.txt)${b:+ (dynamic frames off)}"
done
done
# Before the fix: the v1 source (commit DDE_EMU_V1, from git when available) must FAIL
# the run1 replay and reproduce the car-test-1 console line (FAULT 1, 0x3D never sent).
DDE_EMU_V1="${DDE_EMU_V1:-954110e}"
if git -C ../.. cat-file -e "$DDE_EMU_V1:src/dde_emu.c" 2>/dev/null; then
    V1=/tmp/pico_flexray_dde_emu_v1src
    rm -rf "$V1" && mkdir -p "$V1"
    for f in dde_emu.c dde_emu.h dde_emu_core.c dde_emu_core.h dde_emu_tables.h; do
        git -C ../.. show "$DDE_EMU_V1:src/$f" > "$V1/$f"
    done
    # v1 wrote the FIFO register directly; route it through pio_sm_put (same SDK effect)
    sed -i -E 's/DDE_EMU_PIO->txf\[(sm_[a-z]+)\] = ([^;]+);/pio_sm_put(DDE_EMU_PIO, \1, \2);/' "$V1/dde_emu.c"
    gcc -std=c11 -Wall -Wextra -Werror -Iemu_stubs -I"$V1" -I../../src -o /tmp/pico_flexray_test_dde_emu_replay_v1 \
        test_dde_emu_replay.c "$V1/dde_emu.c" "$V1/dde_emu_core.c" -lm
    if /tmp/pico_flexray_test_dde_emu_replay_v1 emu_run1_fr1_1600ms.txt run1 > /tmp/pico_flexray_replay_v1.txt; then
        echo "v1 source PASSED the run1 replay: the test does not catch the car-test-1 bug"; exit 1
    fi
    echo "v1 source fails the run1 replay (expected): $(grep 'end:' /tmp/pico_flexray_replay_v1.txt | head -n 1)"
    /tmp/pico_flexray_test_dde_emu_replay_v1 emu_run1_fr1_1600ms.txt run1-v1 | tail -n 1
    /tmp/pico_flexray_test_dde_emu_replay_v1 emu_run1_fr1_1600ms.txt run1-v1-spike | tail -n 1
else
    echo "(v1 source $DDE_EMU_V1 not in git here: before-the-fix replay skipped)"
fi
RUN1_CSV="${DDE_RUN1_CSV:-/workspace/f11-emu-run1/f11_emu1.csv}"
if [ -f "$RUN1_CSV" ]; then
    python3 ../../tools/dde_emu/extract_replay.py "$RUN1_CSV" /tmp/pico_flexray_emu_run1_full.txt
    /tmp/pico_flexray_test_dde_emu_replay /tmp/pico_flexray_emu_run1_full.txt run1-full | tail -n 3
else
    echo "(car test 1 capture not found: full 60 s replay skipped)"
fi
# v1c dynamic frames (0xE6/0xE8/0xFB) in the full car runs (captures needed):
#  dyn-run1 / dyn-run2: emulator against the car-test-1/2 vehicle bus (no DDE): grid,
#     minislot order vs the vehicle frames, bytes, counters, periods, pre-ign statics.
#  dyn-unsure: run 2 with injected unexplained frame starts / unknown-length frames /
#     missing anchors: the frame must be skipped in exactly those cycles, then resume.
#  dyn-run5: emulator against run 5 (real DDE on FR2): same minislot/time as the real DDE.
RUN2_CSV="${DDE_RUN2_CSV:-/workspace/f11-emu-run2/f11_emu2.csv}"
if [ -f "$RUN1_CSV" ]; then
    /tmp/pico_flexray_test_dde_emu_replay /tmp/pico_flexray_emu_run1_full.txt dyn-run1 > /tmp/pico_flexray_replay_dyn1.txt ||
        { cat /tmp/pico_flexray_replay_dyn1.txt; exit 1; }
    tail -n 1 /tmp/pico_flexray_replay_dyn1.txt
fi
if [ -f "$RUN2_CSV" ]; then
    python3 ../../tools/dde_emu/extract_replay.py "$RUN2_CSV" /tmp/pico_flexray_emu_run2_full.txt
    for sc in dyn-run2 dyn-unsure; do
        /tmp/pico_flexray_test_dde_emu_replay /tmp/pico_flexray_emu_run2_full.txt $sc > /tmp/pico_flexray_replay_$sc.txt ||
            { cat /tmp/pico_flexray_replay_$sc.txt; exit 1; }
        tail -n 1 /tmp/pico_flexray_replay_$sc.txt
    done
else
    echo "(car test 2 capture not found: dyn-run2 / dyn-unsure skipped)"
fi
if [ -f "$RUN5_CSV" ]; then
    python3 ../../tools/dde_emu/extract_replay.py "$RUN5_CSV" /tmp/pico_flexray_run5_full.txt --channels FR1 \
        --dde-out /tmp/pico_flexray_run5_dde.txt
    /tmp/pico_flexray_test_dde_emu_replay /tmp/pico_flexray_run5_full.txt dyn-run5 /tmp/pico_flexray_run5_dde.txt \
        > /tmp/pico_flexray_replay_dyn5.txt || { cat /tmp/pico_flexray_replay_dyn5.txt; exit 1; }
    grep -E "^  (real|compared|ours)" /tmp/pico_flexray_replay_dyn5.txt | head -n 6 || true
    tail -n 1 /tmp/pico_flexray_replay_dyn5.txt
else
    echo "(run 5 capture not found: dyn-run5 skipped)"
fi
echo "host tests: all OK"
