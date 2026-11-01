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
    test_dde_emu_core.c ../../src/dde_emu_core.c ../../src/dde_engine.c ../../src/dde_gear.c -lm
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
# v1g fake engine start (src/dde_engine.c): off = identical bytes, CAS / fallback start,
# run, stop timelines, every E2E/frame CRC, PT-CAN via pte_next. Timeline CSV kept in /tmp.
gcc -std=c11 -Wall -Wextra -Werror -Istubs -I../../src -o /tmp/pico_flexray_test_engine \
    test_engine.c ../../src/dde_engine.c ../../src/dde_gear.c ../../src/dde_emu_core.c ../../src/ptcan_emu_core.c -lm
/tmp/pico_flexray_test_engine /tmp/pico_flexray_engine_timeline.csv > /tmp/pico_flexray_engine.txt ||
    { cat /tmp/pico_flexray_engine.txt; exit 1; }
tail -n 1 /tmp/pico_flexray_engine.txt

# DDE emulator firmware (src/dde_emu.c, the real file) replayed against car test 1's
# captured FR1 timing (emu_run1_fr1_1600ms.txt, from f11_emu1.csv) with a model of
# TIMER0, the PIO TX/fire SMs and the transceiver echo. See test_dde_emu_replay.c.
# Two builds: the release build (v1d: 0x5C NM, 0xE6/0xE8/0xFB, 0x10A/0x142/0x68, KL15 from
# 0xF1 on) and all dynamic frames off (= v1b behaviour + pre-ign statics) - the v1b
# scenarios must pass on both.
gcc -std=c11 -Wall -Wextra -Werror -Iemu_stubs -I../../src -o /tmp/pico_flexray_test_dde_emu_replay \
    test_dde_emu_replay.c ../../src/dde_emu.c ../../src/dde_emu_core.c ../../src/dde_engine.c ../../src/dde_gear.c -lm
gcc -std=c11 -Wall -Wextra -Werror -Iemu_stubs -I../../src -DEMU_DYN_E6=0 -DEMU_DYN_FB=0 -DEMU_DYN_E8=0 \
    -DEMU_DYN_5C=0 -DEMU_142=0 -DEMU_10A=0 -DEMU_68=0 \
    -o /tmp/pico_flexray_test_dde_emu_replay_nodyn test_dde_emu_replay.c ../../src/dde_emu.c ../../src/dde_emu_core.c ../../src/dde_engine.c ../../src/dde_gear.c -lm
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
    for sc in dyn-run2 dyn-unsure dyn-engine; do
        /tmp/pico_flexray_test_dde_emu_replay /tmp/pico_flexray_emu_run2_full.txt $sc > /tmp/pico_flexray_replay_$sc.txt ||
            { cat /tmp/pico_flexray_replay_$sc.txt; exit 1; }
        tail -n 1 /tmp/pico_flexray_replay_$sc.txt
    done
    # v1h.1 STANDALONE build (-DEMU_STANDALONE=1): no host calls at all; the board's own
    # core-0 poll must switch the emulator + fake start (cas, short) + gear on by itself,
    # and the whole dyn-engine scenario must pass the same checks as with the logger.
    gcc -std=c11 -Wall -Wextra -Werror -Iemu_stubs -I../../src -DEMU_STANDALONE=1 -o /tmp/pico_flexray_test_dde_emu_replay_sa \
        test_dde_emu_replay.c ../../src/dde_emu.c ../../src/dde_emu_core.c ../../src/dde_engine.c ../../src/dde_gear.c \
        ../../src/dde_standalone.c -lm
    /tmp/pico_flexray_test_dde_emu_replay_sa /tmp/pico_flexray_emu_run2_full.txt dyn-engine > /tmp/pico_flexray_replay_standalone.txt ||
        { cat /tmp/pico_flexray_replay_standalone.txt; exit 1; }
    echo "standalone (no host): $(grep '^  standalone:' /tmp/pico_flexray_replay_standalone.txt | cut -c15-) - $(tail -n 1 /tmp/pico_flexray_replay_standalone.txt)"
    # v1h: the other start profiles (0 = build default = short, 2 realistic, 3 run8)
    for prof in 2 3; do
        DDE_ENGINE_PROFILE=$prof /tmp/pico_flexray_test_dde_emu_replay /tmp/pico_flexray_emu_run2_full.txt dyn-engine \
            > /tmp/pico_flexray_replay_dyn-engine_p$prof.txt || { cat /tmp/pico_flexray_replay_dyn-engine_p$prof.txt; exit 1; }
        echo "dyn-engine profile $prof: $(grep '^  engine: rpm' /tmp/pico_flexray_replay_dyn-engine_p$prof.txt | cut -c1-120)"
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
# v1h.1 gear shifting (src/dde_gear.c): lever table, refusals, timings, lever timeout,
# gear off = v1h bytes, PT-CAN/FlexRay bytes + CRCs, and the real r10md10m lever/brake
# replayed against the real EGS 0x3FD. Timeline CSV kept in /tmp.
gcc -std=c11 -Wall -Wextra -Werror -Istubs -I../../src -o /tmp/pico_flexray_test_gear \
    test_gear.c ../../src/dde_engine.c ../../src/dde_gear.c ../../src/dde_emu_core.c ../../src/ptcan_emu_core.c -lm
/tmp/pico_flexray_test_gear /tmp/pico_flexray_gear_timeline.csv gear_vectors_r10md10m.txt /tmp/pico_flexray_gearsim.csv.can.csv \
    > /tmp/pico_flexray_gear.txt || { cat /tmp/pico_flexray_gear.txt; exit 1; }
grep "matched within" /tmp/pico_flexray_gear.txt
tail -n 1 /tmp/pico_flexray_gear.txt
# checker --gear on the simulated shift sequence (PT-CAN only, no console log): the three
# gear PASS lines, both refusals shown as "no change", every accepted push answered
python3 ../../tools/dde_emu/check_emu_capture.py /tmp/pico_flexray_gearsim.csv --engine --gear \
    > /tmp/pico_flexray_gearsim_check.txt || true
if [ "$(sed -n '/^5\./,$p' /tmp/pico_flexray_gearsim_check.txt | grep -c '^  PASS')" != 3 ] ||
   [ "$(grep -c 'no change (refused' /tmp/pico_flexray_gearsim_check.txt)" != 2 ]; then
    cat /tmp/pico_flexray_gearsim_check.txt; exit 1
fi
echo "checker --gear on the simulated shifts: 3 gear PASS lines, 2 refusals, $(grep -c '0x3FD [PRND] +' /tmp/pico_flexray_gearsim_check.txt) pushes answered"
# v1h.2: single-frame flicker checker (0x39A/0x3FD/0x3F9/0x0DC/0x397, no settle window):
#  - the simulated shift sequence: 0 flickers
#  - the real run 9 PT-CAN inputs replayed through the model with injected snapshot read
#    failures: v1h.1 behaviour (fall back to stock) must FAIL, v1h.2 (reuse the last good
#    snapshot, the firmware's eng_snap_select) must have 0 flickers at 3 % and 30 % failures
python3 ../../tools/dde_emu/check_emu_capture.py /tmp/pico_flexray_gearsim.csv --flicker > /tmp/pico_flexray_gearsim_flicker.txt || true
if [ "$(sed -n '/^6\./,$p' /tmp/pico_flexray_gearsim_flicker.txt | grep -c '^  PASS')" != 5 ]; then
    cat /tmp/pico_flexray_gearsim_flicker.txt; exit 1
fi
gcc -std=c11 -Wall -Wextra -Werror -Istubs -I../../src -o /tmp/pico_flexray_run9_flicker \
    test_run9_flicker.c ../../src/dde_engine.c ../../src/dde_gear.c ../../src/ptcan_emu_core.c -lm
zcat run9_ptcan_rx.txt.gz > /tmp/pico_flexray_run9_rx.txt
for m in "v1h1 30" "v1h2 30" "v1h2 300"; do
    set -- $m
    /tmp/pico_flexray_run9_flicker /tmp/pico_flexray_run9_rx.txt /tmp/pico_flexray_run9_$1_$2.can.csv $1 $2 > /tmp/pico_flexray_run9_$1_$2.txt
    python3 ../../tools/dde_emu/check_emu_capture.py /tmp/pico_flexray_run9_$1_$2.can.csv --flicker > /tmp/pico_flexray_run9_$1_$2_check.txt || true
    np=$(sed -n '/^6\./,$p' /tmp/pico_flexray_run9_$1_$2_check.txt | grep -c '^  PASS' || true)
    head -n 1 /tmp/pico_flexray_run9_$1_$2.txt
    sed -n '/^6\./,$p' /tmp/pico_flexray_run9_$1_$2_check.txt | grep '^  [PF]' | sed 's/ (e\.g\..*//'
    if [ "$1" = v1h2 ] && [ "$np" != 5 ]; then echo "run 9 replay v1h.2: flickers left"; exit 1; fi
    if [ "$1" = v1h1 ] && [ "$np" = 5 ]; then echo "run 9 replay v1h.1 model: expected flickers (test is blind)"; exit 1; fi
    grep -q "4 gear changes" /tmp/pico_flexray_run9_$1_$2.txt || { echo "run 9 replay: expected the 4 gear changes of run 9"; cat /tmp/pico_flexray_run9_$1_$2.txt; exit 1; }
done
echo "run 9 replay: v1h.1 snapshot fallback flickers, v1h.2 snapshot reuse 0 flickers (3 % and 30 % read failures)"
# EMU_GEAR=0 build compiles and stays fixed P
gcc -std=c11 -Wall -Wextra -Werror -Istubs -I../../src -DEMU_GEAR=0 -c -o /tmp/pico_flexray_gear0.o ../../src/dde_gear.c
# v1h.1: gear status (64 B, USB 0xab) and the GEAR: texts, C vs frts.py
gcc -std=c11 -Wall -Wextra -Werror -Istubs -I../../src -o /tmp/pico_flexray_gear_dump gear_layout_dump.c ../../src/dde_gear.c
/tmp/pico_flexray_gear_dump /tmp/pico_flexray_gear_texts.txt > /tmp/pico_flexray_gear_layout.bin
python3 - <<'PY'
import sys; sys.path.insert(0, '../../tools/ts_capture'); import frts
d = frts.parse_gear_status(open('/tmp/pico_flexray_gear_layout.bin', 'rb').read())
c_texts = open('/tmp/pico_flexray_gear_texts.txt').read().splitlines()
want = dict(version=1, enabled=3, gear=0, stage=4, brake=1, running=1, ev_seq=4, changes=3, refusals=1, lever_bad=1)
bad = [k for k in want if d.get(k) != want[k]]
py_texts = [frts.gear_ev_text(e) for e in d["ev"]]
bad += [] if py_texts == c_texts else ["texts %r != %r" % (py_texts, c_texts)]
news, seen = frts.gear_news(d, 2)
bad += [] if news == c_texts[-2:] and seen == 4 else ["news %r" % news]
bad += [] if "refused, brake not pressed (P -> R, lever forward 2)" in c_texts[1] else ["refusal text"]
print("v1h.1 gear status layout + texts (C vs frts.py): " + ("ok" if not bad else "MISMATCH " + str(bad)))
sys.exit(1 if bad else 0)
PY
# v1h: engine status v2 (56 B) and echo-miss diag (64 B), C vs frts.py
gcc -std=c11 -Wall -Wextra -Werror -I../../src -Iemu_stubs -o /tmp/pico_flexray_v1h_dump v1h_layout_dump.c
/tmp/pico_flexray_v1h_dump > /tmp/pico_flexray_v1h_layout.bin
python3 - <<'PY'
import sys; sys.path.insert(0, '../../tools/ts_capture'); import frts
b = open('/tmp/pico_flexray_v1h_layout.bin', 'rb').read()
e = frts.parse_engine_status(b[:56]); d = frts.parse_emu_diag(b[56:])
we = dict(version=2, mode=1, phase=2, stkl=0xC, rpm=780, starts=3, seq=99, profile=2, profile_req=1, cas_flags=5,
          out_flags=3, d_to_82_ms=840, d_len_ms=120, d_end_after_82_ms=-720, t_rpm_ms=90, aborts=7)
wd = dict(version=2, lm_slot=0x28, lm_cycle=33, lm_flags=0x12, miss28=343, late3d=4, lm_r_entry=-27, lm_r_final=1234,
          lm_isr_late_us=6, lm_sched_lead_us=21, lm_sched_ref=0x27, lm_hdr_us=9, hdr_isr_max_us=14,
          alarm_isr_max_us=31, miss_resched_close=300, miss_crc_near=2, miss_preempt=5, resched_guard_skips=6,
          crc_fail=487, no_cand=70000, static_fired=123456, hdr_over_10us=8, resched_min_us=40,
          snap_fail=17, bg_max_us=23)
bad = [k for k in we if e.get(k) != we[k]] + [k for k in wd if d.get(k) != wd[k]]
old = frts.parse_engine_status(b[:40])   # a v1g.2-sized read still parses
bad += [] if old and "cas_flags" not in old and old["rpm"] == 780 else ["v1-sized read"]
news = frts.engine_cas_news(e, (3, 0))
bad += [] if any("BEFORE our running report" in m for m in news) and any("300 ms" in m for m in news) else ["cas news"]
print("v1h layouts (engine status v2, emu diag; C vs frts.py): " + ("ok" if not bad else "MISMATCH " + str(bad)))
print("  " + frts.fmt_engine(e)); print("  " + frts.fmt_emu_diag(d))
for m in news: print("  ENGINE: *** %s ***" % m)
sys.exit(1 if bad else 0)
PY
# v1h: USB throughput model for the 1 KB transfers (>= 300 kB/s with headroom)
python3 usb_fast_model.py > /tmp/pico_flexray_usb_model.txt || { cat /tmp/pico_flexray_usb_model.txt; exit 1; }
tail -n 1 /tmp/pico_flexray_usb_model.txt
# v1h: the logger's parser gives the same CSV whatever the USB read boundaries are (1 KB
# transfers split records differently) and parses well above 300 kB/s on this machine.
RUN8_BIN="${RUN8_BIN:-/workspace/f11-emu-run8/f11_emu8_v1g.csv.bin}"
if [ -f "$RUN8_BIN" ]; then
    head -c 4000000 "$RUN8_BIN" > /tmp/pico_flexray_run8_head.bin
    LG=../../tools/ts_capture/fr_ts_logger.py
    for ch in 65536 1024 777; do
        python3 $LG --replay /tmp/pico_flexray_run8_head.bin -o /tmp/pico_flexray_rp_$ch.csv --can --no-console-log \
            --replay-chunk $ch > /dev/null || exit 1
    done
    cmp -s /tmp/pico_flexray_rp_65536.csv /tmp/pico_flexray_rp_1024.csv && cmp -s /tmp/pico_flexray_rp_65536.csv /tmp/pico_flexray_rp_777.csv &&
        cmp -s /tmp/pico_flexray_rp_65536.csv.can.csv /tmp/pico_flexray_rp_777.csv.can.csv ||
        { echo "logger output depends on the USB read boundaries"; exit 1; }
    T0=$(date +%s.%N)
    python3 $LG --replay /tmp/pico_flexray_run8_head.bin -o /tmp/pico_flexray_rp_bench.csv --can --no-console-log > /dev/null
    T1=$(date +%s.%N)
    python3 -c "import sys; r=4000000/($T1-$T0)/1e3; print('logger parse: same CSV for 64 KB / 1 KB / 777 B reads; %.0f kB/s (%.0fx 300 kB/s)' % (r, r/300)); sys.exit(0 if r >= 900 else 1)" || exit 1
else
    echo "(run 8 raw dump not found: logger chunking / speed check skipped)"
fi
# v1e PT-CAN DDE/EGS emulation (src/ptcan_emu_core.c): unit checks, E2E CRC/counter of
# real frames (committed vector set; whole run 4 capture if present), gate / timeline
# replays against run 4 (DDE+EGS on the bus, and DDE unplugged) and run 5 (EGS fuse pulled,
# DDE off = the v1e target setup), ptcan_stats_t layout C vs frts.py, logger --can-tx flow.
gcc -std=c11 -Wall -Wextra -Werror -Istubs -I../../src -o /tmp/pico_flexray_test_pte \
    test_ptcan_emu.c ../../src/ptcan_emu_core.c ../../src/dde_engine.c ../../src/dde_gear.c
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
# v1f: run 6 (full emulation) rate check - every 20/30/100 ms slot sent, with and without
# the v1e main-loop stall modelled (150 ms every 5 s). Before the fix: the v1e core (git
# PTE_V1E, 4 frames per pass) with the same stall must reproduce the run 6 loss
# (0x0A5 ~6,446 of 6,628 vs 6,447 of 6,634 in the car).
RUN6_FULLEMU="${PTE_RUN6_FULLEMU:-/workspace/f11-emu-run6/replay/f11_emu6_fullemu.csv.can.csv}"
if [ -f "$RUN6_FULLEMU" ]; then
    python3 $CHK /tmp/pico_flexray_test_pte rate "$RUN6_FULLEMU" > /tmp/pico_flexray_pte_rate.txt ||
        { cat /tmp/pico_flexray_pte_rate.txt; exit 1; }
    tail -n 1 /tmp/pico_flexray_pte_rate.txt
    PTE_V1E="${PTE_V1E:-d7e490a}"
    if git -C ../.. cat-file -e "$PTE_V1E:src/ptcan_emu_core.c" 2>/dev/null; then
        V1E=/tmp/pico_flexray_pte_v1esrc
        mkdir -p $V1E
        git -C ../.. show "$PTE_V1E:src/ptcan_emu_core.c" > $V1E/ptcan_emu_core.c
        git -C ../.. show "$PTE_V1E:src/ptcan_emu_core.h" > $V1E/ptcan_emu_core.h
        gcc -std=c11 -Wall -Wextra -Werror -DPTE_V1E_COMPARE -Istubs -I$V1E -I../../src \
            -o /tmp/pico_flexray_test_pte_v1e test_ptcan_emu.c $V1E/ptcan_emu_core.c
        if python3 $CHK /tmp/pico_flexray_test_pte_v1e rate "$RUN6_FULLEMU" > /tmp/pico_flexray_pte_rate_v1e.txt; then
            echo "v1e core unexpectedly passed the stall rate check"; exit 1
        fi
        echo "v1e core (before the fix), stall modelled: $(grep 'rate_stall_burst4: 0x0a5' /tmp/pico_flexray_pte_rate_v1e.txt | sed 's/^ *FAIL *//')"
    fi
else
    echo "(run 6 capture not found: rate check skipped)"
fi
/tmp/pico_flexray_test_pte stats /tmp/pico_flexray_pte_stats.bin > /dev/null
python3 test_cantx_logger.py /tmp/pico_flexray_pte_stats.bin > /tmp/pico_flexray_cantx_logger.txt 2>&1 ||
    { cat /tmp/pico_flexray_cantx_logger.txt; exit 1; }
tail -n 1 /tmp/pico_flexray_cantx_logger.txt
# v1g: check_emu_capture.py --engine on a real DDE start (SavvyCAN FR capture): the
# per-start report, E2E CRC checks and "0x82 before CAS releases the starter" must pass
ENGSTART=${ENGSTART_CSV:-/workspace/flexray-re-notes/captures/F11_engstartthenstop.csv}
if [ -f "$ENGSTART" ]; then
    python3 savvy_fr_to_frts.py "$ENGSTART" /tmp/pico_flexray_engstart.csv > /dev/null
    python3 ../../tools/dde_emu/check_emu_capture.py /tmp/pico_flexray_engstart.csv --engine \
        > /tmp/pico_flexray_engstart_check.txt || true
    if grep -q "FAIL.*\(0x82\|E2E\|0xE6 b3\|ST_KL reached\)" /tmp/pico_flexray_engstart_check.txt ||
       ! grep -q "PASS 0x82 sent before CAS released" /tmp/pico_flexray_engstart_check.txt; then
        cat /tmp/pico_flexray_engstart_check.txt; exit 1
    fi
    echo "checker --engine on the real start capture: $(grep '^  start at' /tmp/pico_flexray_engstart_check.txt | head -n 1)"
else
    echo "(real engine start capture not found: checker --engine test skipped)"
fi
# v1h: the checker on run 8 (v1g.2, accepted as a pass): both 0x397 lines (stock spacing
# while stopped, 60 ms while running) must pass; the only FAIL left is the real USB loss.
RUN8_DIR=${RUN8_DIR:-/workspace/f11-emu-run8}
if [ -f "$RUN8_DIR/replay/f11_emu8_v1g.csv" ]; then
    python3 ../../tools/dde_emu/check_emu_capture.py "$RUN8_DIR/replay/f11_emu8_v1g.csv" --engine \
        --console "$RUN8_DIR/input/f11_emu8_v1g.console.log" > /tmp/pico_flexray_run8_check.txt || true
    if [ "$(grep -c '^  FAIL' /tmp/pico_flexray_run8_check.txt)" != 1 ] ||
       ! grep -q "FAIL FlexRay frames lost before USB" /tmp/pico_flexray_run8_check.txt ||
       [ "$(grep -c 'PASS 0x397 engine' /tmp/pico_flexray_run8_check.txt)" != 2 ]; then
        cat /tmp/pico_flexray_run8_check.txt; exit 1
    fi
    echo "checker on run 8: 0x397 stopped + running PASS, only FAIL = USB loss; $(grep 'latency after' /tmp/pico_flexray_run8_check.txt)"
else
    echo "(run 8 replay not found: checker run 8 test skipped)"
fi
# v1g.2: the real src/ptcan_emu.c with a pretend CAN chip and clock (test_ptcan_tef.c):
# own-TX log reads (TEF) only when the host asked, only for frames still outstanding,
# at most once per ms; every sent frame logged once with its exact start-of-frame time.
TEFCF="-std=gnu11 -Wall -Wextra -Werror -DPICO_FLEXRAY_BOARD_V2=1 -DPICO_FLEXRAY_DDE_EMU=1 -DEMU_CAN_TX=1 -Itef_stubs -Istubs -I../../src"
gcc $TEFCF -o /tmp/pico_flexray_test_tef test_ptcan_tef.c ../../src/ptcan_emu.c ../../src/ptcan_emu_core.c \
    ../../src/dde_engine.c ../../src/dde_gear.c -lm
for sc in log-on log-off toggle noack; do
    /tmp/pico_flexray_test_tef $sc 20 > /dev/null 2> /tmp/pico_flexray_tef_$sc.txt ||
        { cat /tmp/pico_flexray_tef_$sc.txt; exit 1; }
    head -n 1 /tmp/pico_flexray_tef_$sc.txt | grep -v "after the" || sed -n 2p /tmp/pico_flexray_tef_$sc.txt
done
# Before the fix (v1g 58cf388 ptcan_emu.c, TEF read on every pass): numbers only.
TEF_OLD="${TEF_OLD:-58cf388}"
if git -C ../.. cat-file -e "$TEF_OLD:src/ptcan_emu.c" 2>/dev/null; then
    git -C ../.. show "$TEF_OLD:src/ptcan_emu.c" > /tmp/pico_flexray_ptcan_emu_old.c
    gcc $TEFCF -o /tmp/pico_flexray_test_tef_old test_ptcan_tef.c /tmp/pico_flexray_ptcan_emu_old.c \
        ../../src/ptcan_emu_core.c ../../src/dde_engine.c ../../src/dde_gear.c -lm
    for sc in log-on log-off; do
        echo "before the fix ($TEF_OLD): $(/tmp/pico_flexray_test_tef_old $sc 20 report 2>&1 > /dev/null)"
    done
fi
echo "host tests: all OK"
