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
gcc -std=c11 -Wall -Wextra -Werror -I../../src -o /tmp/pico_flexray_test_dde_emu \
    test_dde_emu_core.c ../../src/dde_emu_core.c
RUN5_BIN="${DDE_RUN5_BIN:-/workspace/f11-ts-run5/f11_run5.csv.bin}"
if [ -f "$RUN5_BIN" ]; then
    python3 ../../tools/dde_emu/gen_dde_tables.py --bin "$RUN5_BIN" --out /tmp/pico_flexray_dde_tables_check.h \
        --vectors /tmp/pico_flexray_dde_vectors_all.txt > /dev/null
    cmp /tmp/pico_flexray_dde_tables_check.h ../../src/dde_emu_tables.h
    echo "dde_emu_tables.h: regenerated from the capture, identical"
    /tmp/pico_flexray_test_dde_emu strict:dde_vectors_run5_2s.txt /tmp/pico_flexray_dde_vectors_all.txt
else
    echo "(run 5 capture not found: full-run vector check skipped)"
    /tmp/pico_flexray_test_dde_emu strict:dde_vectors_run5_2s.txt
fi
# DDE emulator firmware (src/dde_emu.c, the real file) replayed against car test 1's
# captured FR1 timing (emu_run1_fr1_1600ms.txt, from f11_emu1.csv) with a model of
# TIMER0, the PIO TX/fire SMs and the transceiver echo. See test_dde_emu_replay.c.
gcc -std=c11 -Wall -Wextra -Werror -Iemu_stubs -I../../src -o /tmp/pico_flexray_test_dde_emu_replay \
    test_dde_emu_replay.c ../../src/dde_emu.c ../../src/dde_emu_core.c
for sc in run1 run1-spike run1-bigspike lateecho foreign stuck noecho; do
    /tmp/pico_flexray_test_dde_emu_replay emu_run1_fr1_1600ms.txt $sc > /tmp/pico_flexray_replay_$sc.txt ||
        { cat /tmp/pico_flexray_replay_$sc.txt; exit 1; }
    tail -n 1 /tmp/pico_flexray_replay_$sc.txt
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
        test_dde_emu_replay.c "$V1/dde_emu.c" "$V1/dde_emu_core.c"
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
