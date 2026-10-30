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
