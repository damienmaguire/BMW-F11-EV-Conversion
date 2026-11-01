// v1h.2: replay the real run 9 PT-CAN inputs (every Rx frame of f11_emu9_v1h1: KL15 /
// ST_KL 0x12F, brake 0x173, lever 0x197, NM ...) through the host model of the emulator
// (engine story, gear, PT-CAN scheduler) and write our TX as a logger-style CAN CSV, so
// tools/dde_emu/check_emu_capture.py --flicker can count single-frame flickers.
//
// The core1 -> core0 snapshot read is modelled with injected read failures (default 3 %
// of the reads, far more than run 9's ~1 %):
//   mode v1h1: a failed read = no snapshot = stock template (fixed P, engine off) for
//              that PT-CAN pass  -> the run 9 flickers;
//   mode v1h2: eng_snap_select() (the firmware's code) reuses the last good snapshot.
//
//   test_run9_flicker RX.txt OUT.can.csv v1h1|v1h2 [fail_permille]
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dde_engine.h"
#include "dde_gear.h"
#include "ptcan_emu_core.h"

static eng_t E;
static gear_t G;
static pte_t P;
static FILE *out;
static uint32_t rng = 12345u;
static uint32_t rnd(void) { rng = rng * 1103515245u + 12345u; return (rng >> 8) & 0xFFFFu; }

static void row(uint64_t t, uint16_t id, const uint8_t *d, unsigned n, bool tx)
{
    fprintf(out, "%llu,0x%X,0,", (unsigned long long)t, id);
    for (unsigned i = 0; i < n; i++) fprintf(out, "%02x", d[i]);
    fprintf(out, ",%s\n", tx ? "Tx" : "Rx");
}

int main(int argc, char **argv)
{
    if (argc < 4) { fprintf(stderr, "usage: %s RX.txt OUT.can.csv v1h1|v1h2 [fail_permille]\n", argv[0]); return 2; }
    FILE *f = fopen(argv[1], "r");
    out = fopen(argv[2], "w");
    if (!f || !out) { perror("open"); return 2; }
    const bool v1h2 = !strcmp(argv[3], "v1h2");
    const unsigned fail_pm = argc > 4 ? (unsigned)atoi(argv[4]) : 30u;
    fprintf(out, "t_us,can_id,ide,data_hex,dir\n");
    pte_init(&P);
    pte_set_mask(&P, PTE_MASK_ALL);
    memset(&E, 0, sizeof E);
    eng_reset(&E);
    memset(&G, 0, sizeof G);
    gear_reset(&G, true);
    eng_snap_cache_t cache; memset(&cache, 0, sizeof cache);
    const uint64_t T0 = 10000000ull;   // model time = capture time + 10 s
    uint64_t T = T0, next_ms = T0;
    unsigned long n_rx = 0, n_tx = 0, n_fail = 0, n_stock = 0, starts_seen = 0;
    char line[160];
    bool more = true;
    unsigned long t; unsigned id; char hex[40];
    bool have = false;
    while (more) {
        if (!have) {
            more = false;
            while (fgets(line, sizeof line, f)) {
                if (line[0] == '#') continue;
                if (sscanf(line, "%lu %x %39s", &t, &id, hex) == 3) { have = more = true; break; }
            }
            if (!more) break;
        }
        uint64_t tr = T0 + t;
        // 1 ms model steps up to this frame
        while (next_ms <= tr) {
            T = next_ms;
            uint64_t ms = (T - T0) / 1000u;
            if (ms % 5 == 0) eng_step(&E, (uint32_t)T);   // FlexRay cycle
            gear_step(&G, (uint32_t)T, E.phase == ENG_RUN);   // v1h.2: every 1 ms
            gear_fill_out(&G, &E.out);
            // core 0's snapshot read (failures injected)
            bool ok = E.out.active && (rnd() % 1000u) >= fail_pm;
            if (E.out.active && !ok) n_fail++;
            const eng_out_t *s;
            if (v1h2) s = eng_snap_select(&cache, E.out.active ? ok : true, &E.out, (uint32_t)T, (uint32_t)T);
            else s = ok ? &E.out : NULL;
            if (E.out.active && !s) n_stock++;
            P.eng = s;
            pte_tx_t tx;
            while (pte_next(&P, T, &tx)) {
                row(T, tx.id, tx.data, tx.dlc, true);
                pte_commit(&P, &tx, T);
                n_tx++;
            }
            next_ms += 1000u;
        }
        uint8_t d[8] = {0}; unsigned n = 0;
        for (; n < 8 && hex[2 * n] && hex[2 * n + 1]; n++) sscanf(hex + 2 * n, "%2hhx", &d[n]);
        T = tr;
        pte_on_rx(&P, (uint16_t)id, d, (uint8_t)n, T);
        if (id == 0x12F && n > 6) {
            eng_set_mode(&E, ENG_MODE_CAS, (uint32_t)T);
            eng_input_stkl(&E, d[2] & 15u, (d[6] & 3u) == 3u, (uint32_t)T);
        }
        if (id == 0x173 && n > 7 && (d[7] & 3u) == 3u) { eng_input_brake(&E, true, (uint32_t)T); gear_input_brake(&G, (uint32_t)T); }
        if (id == 0x197) gear_input_lever(&G, d, (uint8_t)n, (uint32_t)T);
        if (id == 0x197 || id == 0x173 || id == 0x12F) row(T, (uint16_t)id, d, n, false);
        n_rx++;
        have = false;
    }
    starts_seen = E.starts;
    printf("run 9 replay (%s, %u per mille read failures): %lu Rx in, %lu Tx out, %lu failed reads, "
           "%lu PT-CAN passes on the stock template while emulating, %lu starts, %u gear changes, %u refusals\n",
           v1h2 ? "v1h.2 snapshot reuse" : "v1h.1 fallback to stock", fail_pm, n_rx, n_tx, n_fail, n_stock,
           starts_seen, G.changes, G.refusals);
    for (unsigned i = 0; i < GEAR_N_EV; i++) if (G.ev[i].seq) { char b[96]; gear_ev_text(&G.ev[i], b, sizeof b); printf("  %s\n", b); }
    fclose(out); fclose(f);
    return 0;
}
