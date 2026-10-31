#!/bin/sh
# Host unit checks for the FIXED ts-capture build (pass-through repeater):
# TS-CAPTURE record encoder / timestamp matching, MCP251863 (V2 PT-CAN RX) driver checks.
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
# Two builds: the release build (v1d: 0x5C NM, 0xE6/0xE8/0xFB, 0x10A/0x142/0x68, KL15 from
# 0xF1 on) and all dynamic frames off (= v1b behaviour + pre-ign statics) - the v1b
# scenarios must pass on both.
gcc -std=c11 -Wall -Wextra -Werror -Iemu_stubs -I../../src -o /tmp/pico_flexray_test_dde_emu_replay \
    test_dde_emu_replay.c ../../src/dde_emu.c ../../src/dde_emu_core.c -lm
gcc -std=c11 -Wall -Wextra -Werror -Iemu_stubs -I../../src -DEMU_DYN_E6=0 -DEMU_DYN_FB=0 -DEMU_DYN_E8=0 \
    -DEMU_DYN_5C=0 -DEMU_142=0 -DEMU_10A=0 -DEMU_68=0 \
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
# v1d, all of them: 0x10A/0x142/0x68 grid/order/content (0x142 phases), KL15 gating, and the
# 0x5C NM frame against an independent KL15/NM model of the vehicle frames (0xF1 b3, 0x103).
#  dyn-run3: car test 3 (v1c run, ign on / off, bus to sleep): 0x5C ends KL15 off + 47 s.
#  dyn-mitm: MITM ign on/off capture (SavvyCAN, lossy vehicle side): KL15/NM vs the real DDE.
RUN2_CSV="${DDE_RUN2_CSV:-/workspace/f11-emu-run2/f11_emu2.csv}"
RUN3_CSV="${DDE_RUN3_CSV:-/workspace/f11-emu-run3/f11_emu3.csv}"
MITM_CSV="${DDE_MITM_CSV:-/workspace/flexray-re-notes/captures/mitm/F11_mitm_ignonoff.csv}"
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
    grep -E "^  (real|compared|ours|vs real|NM 0x5C)" /tmp/pico_flexray_replay_dyn5.txt | head -n 8 || true
    tail -n 1 /tmp/pico_flexray_replay_dyn5.txt
else
    echo "(run 5 capture not found: dyn-run5 skipped)"
fi
if [ -f "$RUN3_CSV" ]; then
    python3 ../../tools/dde_emu/extract_replay.py "$RUN3_CSV" /tmp/pico_flexray_emu_run3_full.txt
    /tmp/pico_flexray_test_dde_emu_replay /tmp/pico_flexray_emu_run3_full.txt dyn-run3 > /tmp/pico_flexray_replay_dyn3.txt ||
        { cat /tmp/pico_flexray_replay_dyn3.txt; exit 1; }
    grep -E "^  NM 0x5C" /tmp/pico_flexray_replay_dyn3.txt || true
    tail -n 1 /tmp/pico_flexray_replay_dyn3.txt
else
    echo "(car test 3 capture not found: dyn-run3 skipped)"
fi
if [ -f "$MITM_CSV" ]; then
    python3 ../../tools/dde_emu/savvy_to_replay.py "$MITM_CSV" /tmp/pico_flexray_mitm.txt --dde-out /tmp/pico_flexray_mitm_dde.txt \
        --start-s 7.05
    /tmp/pico_flexray_test_dde_emu_replay /tmp/pico_flexray_mitm.txt dyn-mitm /tmp/pico_flexray_mitm_dde.txt \
        > /tmp/pico_flexray_replay_mitm.txt || { cat /tmp/pico_flexray_replay_mitm.txt; exit 1; }
    grep -E "^  NM 0x5C" /tmp/pico_flexray_replay_mitm.txt || true
    tail -n 1 /tmp/pico_flexray_replay_mitm.txt
else
    echo "(MITM ign on/off capture not found: dyn-mitm skipped)"
fi
# v1d: frts.py EMU_STATS_V4_STRUCT vs the C dde_emu_stats_t (256 bytes)
gcc -std=c11 -Wall -Wextra -Werror -I../../src -Iemu_stubs -o /tmp/pico_flexray_stats_dump stats_layout_dump.c
/tmp/pico_flexray_stats_dump > /tmp/pico_flexray_stats_v4.bin
python3 - <<'PY'
import sys; sys.path.insert(0, '../../tools/ts_capture'); import frts
d = frts.parse_emu_stats(open('/tmp/pico_flexray_stats_v4.bin', 'rb').read())
want = dict(v4=True, dyn_sent2=[11, 22, 33], dyn_skip2=[0, 0, 7], dyn_last_ms2=[129, 0, 0], kl15=0x55, f1_b3=0xE0,
            kl15_on_edges=2, kl15_off_edges=3, nm_ceiling_stops=4, kl15_off_ago_ms=5000, ceil_left_ms=6000,
            isr_late_min_us=-2, isr_late_max_us=9, r_entry_min=-5, r_final_min=77, topup_timeouts=8,
            hk_deferred=65535, dyn_flags2=0x1B, ph142=1, abs_cycle=123456)
bad = [k for k in want if d.get(k) != want[k]]
print("stats v4 layout (C vs frts.py): " + ("ok" if not bad else "MISMATCH " + str(bad)))
sys.exit(1 if bad else 0)
PY
# v1e PT-CAN DDE/EGS emulation (src/ptcan_emu_core.c): unit checks, E2E CRC/counter of
# real frames (committed vector set; whole run 4 capture if present), gate / timeline
# replays against run 4 (DDE+EGS on the bus, and DDE unplugged) and run 5 (EGS fuse pulled,
# DDE off = the v1e target setup), ptcan_stats_t layout C vs frts.py, logger --can-tx flow.
gcc -std=c11 -Wall -Wextra -Werror -Istubs -I../../src -o /tmp/pico_flexray_test_pte \
    test_ptcan_emu.c ../../src/ptcan_emu_core.c
/tmp/pico_flexray_test_pte unit
/tmp/pico_flexray_test_pte e2e ptcan_e2e_vectors_run4.txt
RUN4_DDECAN="${PTE_RUN4_DDECAN:-/workspace/f11-emu-run4/replay/f11_emu4_ddecan.csv.can.csv}"
RUN4_NODDECAN="${PTE_RUN4_NODDECAN:-/workspace/f11-emu-run4/f11_emu4_noddecan.csv.can.csv}"
RUN5_NOEGS="${PTE_RUN5_NOEGS:-/workspace/f11-emu-run5/replay/f11_emu5_noegs.csv.can.csv}"
CHK=../../tools/dde_emu/check_ptcan_replay.py
if [ -f "$RUN4_DDECAN" ]; then
    python3 $CHK /tmp/pico_flexray_test_pte vectors "$RUN4_DDECAN" /tmp/pico_flexray_pte_vectors_all.txt > /dev/null
    /tmp/pico_flexray_test_pte e2e /tmp/pico_flexray_pte_vectors_all.txt
    python3 $CHK /tmp/pico_flexray_test_pte ddecan "$RUN4_DDECAN" > /tmp/pico_flexray_pte_ddecan.txt ||
        { cat /tmp/pico_flexray_pte_ddecan.txt; exit 1; }
    tail -n 1 /tmp/pico_flexray_pte_ddecan.txt
else
    echo "(run 4 ddecan CAN capture not found: PT-CAN replay skipped)"
fi
if [ -f "$RUN4_NODDECAN" ]; then
    python3 $CHK /tmp/pico_flexray_test_pte noddecan "$RUN4_NODDECAN" > /tmp/pico_flexray_pte_noddecan.txt ||
        { cat /tmp/pico_flexray_pte_noddecan.txt; exit 1; }
    tail -n 1 /tmp/pico_flexray_pte_noddecan.txt
fi
if [ -f "$RUN5_NOEGS" ]; then
    python3 $CHK /tmp/pico_flexray_test_pte noegs "$RUN5_NOEGS" > /tmp/pico_flexray_pte_noegs.txt ||
        { cat /tmp/pico_flexray_pte_noegs.txt; exit 1; }
    tail -n 1 /tmp/pico_flexray_pte_noegs.txt
fi
/tmp/pico_flexray_test_pte stats /tmp/pico_flexray_pte_stats.bin > /dev/null
python3 test_cantx_logger.py /tmp/pico_flexray_pte_stats.bin > /tmp/pico_flexray_cantx_logger.txt 2>&1 ||
    { cat /tmp/pico_flexray_cantx_logger.txt; exit 1; }
tail -n 1 /tmp/pico_flexray_cantx_logger.txt
echo "host tests: all OK"
