// Host test of the DDE emulator core (src/dde_emu_core.c) against captured
// run 5 DDE frames (text vectors: "id cycle rawhex"), plus schedule maths and
// the reference-lock state machine.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "dde_emu_core.h"
#include "dde_emu_tables.h"

static int fails;
#define CHECK(c, ...) do { if (!(c)) { fails++; if (fails < 20) { printf("FAIL %s:%d: ", __FILE__, __LINE__); printf(__VA_ARGS__); printf("\n"); } } } while (0)

static int hexval(char c) { return c <= '9' ? c - '0' : (c | 0x20) - 'a' + 10; }

static void test_vectors(const char *path, bool strict)
{
    FILE *f = fopen(path, "r");
    if (!f) { printf("FAIL: cannot open %s\n", path); fails++; return; }
    char line[512];
    unsigned n_rebuild_diff = 0;
    unsigned n = 0, n_hcrc = 0, n_fcrc = 0, n_e2e = 0, n_build = 0, n_dyn = 0, n_pack = 0;
    while (fgets(line, sizeof line, f)) {
        if (line[0] == '#') continue;
        unsigned id, cycle;
        char hex[400];
        if (sscanf(line, "%u %u %399s", &id, &cycle, hex) != 3) continue;
        uint8_t raw[200];
        unsigned len = (unsigned)strlen(hex) / 2;
        for (unsigned i = 0; i < len; i++) raw[i] = (uint8_t)(hexval(hex[2 * i]) << 4 | hexval(hex[2 * i + 1]));
        n++;
        unsigned ind = raw[0] >> 3, lw = raw[2] >> 1;
        unsigned hc = ((raw[2] & 1u) << 10) | ((unsigned)raw[3] << 2) | (raw[4] >> 6);
        CHECK(dde_core_header_crc((ind >> 1) & 1, ind & 1, id, lw) == hc, "hcrc id %u c %u", id, cycle);
        n_hcrc++;
        uint32_t fc = ((uint32_t)raw[len - 3] << 16) | ((uint32_t)raw[len - 2] << 8) | raw[len - 1];
        CHECK(dde_core_frame_crc(raw, len - 3) == fc, "fcrc id %u c %u", id, cycle);
        n_fcrc++;
        int t = dde_core_target_index(id);
        if (t >= 0 && (ind & 4)) {
            // E2E: re-apply with the captured counter -> must reproduce the captured payload
            uint8_t p[16];
            memcpy(p, raw + 5, 16);
            unsigned ctr = 0;
            for (unsigned g = 0; g < DDE_EMU_N_E2E; g++)
                if (dde_emu_e2e[g][0] == t && dde_emu_e2e[g][1] == (cycle & 3)) { ctr = p[dde_emu_e2e[g][2] + 1] & 15; break; }
            uint8_t q[16];
            memcpy(q, p, 16);
            unsigned ng = dde_core_e2e_apply(q, (unsigned)t, cycle, ctr);
            CHECK(ng > 0 && memcmp(p, q, 16) == 0, "e2e id 0x%X c %u", id, cycle);
            n_e2e++;
        }
        if (t >= 0) {
            // full frame from the tables (captured sync bits kept) == captured raw frame
            unsigned ctr = 0;
            for (unsigned g = 0; g < DDE_EMU_N_E2E; g++)
                if (dde_emu_e2e[g][0] == t && dde_emu_e2e[g][1] == (cycle & 3)) { ctr = raw[5 + dde_emu_e2e[g][2] + 1] & 15; break; }
            uint8_t b[DDE_MAX_FRAME_BYTES];
            unsigned bn = dde_core_build_static((unsigned)t, cycle, ctr, true, b);
            bool same = bn == len && memcmp(b, raw, len) == 0;
            if (!same) n_rebuild_diff++;
            CHECK(same || !strict, "build id 0x%X c %u", id, cycle);
            n_build++;
            // emulator header: sync/startup cleared, header CRC recomputed
            unsigned en = dde_core_build_static((unsigned)t, cycle, ctr, false, b);
            unsigned eind = b[0] >> 3;
            unsigned ehc = ((b[2] & 1u) << 10) | ((unsigned)b[3] << 2) | (b[4] >> 6);
            CHECK((eind & 3) == 0 && (!same || (eind & 4) == (ind & 4)), "emu ind id 0x%X", id);
            CHECK(ehc == dde_core_header_crc(0, 0, id, 8), "emu hcrc id 0x%X", id);
            CHECK((b[4] & 63) == cycle, "cycle field");
            CHECK(!same || memcmp(b + 5, raw + 5, 16) == 0, "emu payload id 0x%X c %u", id, cycle);
            uint32_t efc = ((uint32_t)b[en - 3] << 16) | ((uint32_t)b[en - 2] << 8) | b[en - 1];
            CHECK(dde_core_frame_crc(b, en - 3) == efc, "emu fcrc");
            // PIO words round trip
            uint32_t w[DDE_TX_MAX_WORDS];
            unsigned nw = dde_core_pack(b, en, EMU_TSS_BITS, 0, w);
            CHECK(nw == 2 + en / 4 && w[0] == ((uint32_t)(EMU_TSS_BITS - 2) << 16 | (en - 1)) && w[1] == 0, "pack hdr");
            for (unsigned i = 0; i < en; i++) CHECK(((w[2 + i / 4] >> (24 - 8 * (i % 4))) & 0xFF) == b[i], "pack byte");
            n_pack++;
        }
        if (id == DDE_EMU_DYN_ID && dde_core_dyn_cycle(cycle)) {
            uint8_t b[DDE_MAX_FRAME_BYTES];
            unsigned bn = dde_core_build_dyn(cycle, b);
            CHECK(!strict || (bn == len && memcmp(b, raw, len) == 0), "dyn build c %u", cycle);
            uint32_t w[DDE_TX_MAX_WORDS];
            CHECK(dde_core_pack(b, bn, EMU_TSS_BITS, DDE_EMU_DYN_DTS_BITS, w) == 2 + bn / 4 &&
                  w[1] == DDE_EMU_DYN_DTS_BITS - 2, "dyn pack");
            n_dyn++;
        }
    }
    fclose(f);
    printf("%s: %u frames: hcrc %u, fcrc %u, e2e %u, table rebuild %u (%u differ%s), pack %u, dyn 0x5C %u\n",
           path, n, n_hcrc, n_fcrc, n_e2e, n_build, n_rebuild_diff,
           strict ? "" : ": other vehicle states, informational", n_pack, n_dyn);
    CHECK(n > 1000 && n_e2e > 100 && n_build > 500, "too few vectors");
}

static void test_pack_args(void)
{
    uint8_t b[24] = {0};
    uint32_t w[DDE_TX_MAX_WORDS];
    CHECK(dde_core_pack(b, 22, 7, 0, w) == 0, "odd length rejected");
    CHECK(dde_core_pack(b, 24, 2, 0, w) == 0, "tss 2 rejected");
    CHECK(dde_core_pack(b, 24, 16, 0, w) == 0, "tss 16 rejected");
    CHECK(dde_core_pack(b, 24, 7, 2, w) == 0, "dts 2 rejected");
    CHECK(dde_core_pack(b, 36, 7, 0, w) == 0, "too long rejected");
    CHECK(dde_core_pack(b, 24, 7, 0, w) == 8, "24 bytes = 8 words (TX FIFO depth)");
}

static void test_schedule(void)
{
    dde_sched_t s;
    // 0x28 from 0x27: one slot later
    int32_t delta = dde_emu_static_off_q8[1] - dde_emu_ref_off_q8[2];
    dde_core_schedule(0xFF00, 1000000u, delta, 25, 6, &s);
    int32_t dt = (delta + 128) >> 8;
    CHECK(abs(dt - 3300) <= 3, "0x27 -> 0x28 = one slot, 3300 ticks (got %d)", dt);
    CHECK(s.bus_fine == (uint16_t)(0xFF00 + dt), "bus wraps mod 2^16");
    CHECK(s.txen_fine == (uint16_t)(0xFF00 + dt - 25), "txen = bus - loop");
    CHECK(s.alarm_us == 1000000u + 32u - 6u, "alarm us (got %u)", (unsigned)s.alarm_us);
    // a long delta (0x3D from 0x21, 28 slots = 924 us > 655 us fine wrap) still lands right mod 2^16
    delta = dde_emu_static_off_q8[3] - dde_emu_ref_off_q8[1];
    dde_core_schedule(1234, 5u, delta, 0, 6, &s);
    dt = (delta + 128) >> 8;
    CHECK(s.bus_fine == (uint16_t)(1234 + dt), "long delta mod 2^16");
    CHECK(s.alarm_us == 5u + (uint32_t)(dt / 100) - 6u, "long delta alarm");
    // fire delay wraps
    CHECK(dde_core_fire_delay(10, 65530, 30) == 16 - 30, "fire delay across wrap");
    CHECK(dde_core_fire_delay(600, 0, 30) == 570, "fire delay");
    CHECK(dde_core_fire_delay(0, 600, 30) == -630, "late is negative");
    // targets: slot grid sanity (slot ids 38, 40, 41, 61 at 33.00285 us/slot)
    int32_t d26_28 = dde_emu_static_off_q8[1] - dde_emu_static_off_q8[0];
    CHECK(abs(d26_28 - 2 * DDE_EMU_SLOT_Q8) < 256 * 2, "0x26 -> 0x28 = 2 slots");
    int32_t d29_3d = dde_emu_static_off_q8[3] - dde_emu_static_off_q8[2];
    CHECK(abs(d29_3d - 20 * DDE_EMU_SLOT_Q8) < 256 * 4, "0x29 -> 0x3D = 20 slots");
    // every target has a reference at least 20 us before it
    for (int t = 0; t < DDE_EMU_N_STATIC; t++) {
        int ok = 0;
        for (int r = 0; r < DDE_EMU_N_REFS; r++)
            if (dde_emu_static_off_q8[t] - dde_emu_ref_off_q8[r] >= 20 * 100 * 256) ok = 1;
        CHECK(ok, "target %d has a reference", t);
    }
    // no reference is one of our IDs
    for (int r = 0; r < DDE_EMU_N_REFS; r++) CHECK(dde_core_target_index(dde_emu_ref_id[r]) < 0, "ref is target");
}

static void test_lock(void)
{
    dde_lock_t l;
    dde_lock_reset(&l);
    uint32_t t = 100000;
    unsigned c = 10;
    CHECK(dde_lock_on_ref(&l, c, t) == DDE_LOCK_NEW_WAKE, "first ref = wake");
    for (int i = 1; i < 7; i++) CHECK(dde_lock_on_ref(&l, c + i, t + 5000u * i) == DDE_LOCK_NEW_CYCLE, "cycle");
    CHECK(!l.locked, "not locked after 7");
    CHECK(dde_lock_on_ref(&l, c + 6, t + 5000u * 6 + 300) == DDE_LOCK_SAME_CYCLE, "second ref same cycle");
    CHECK(dde_lock_on_ref(&l, c + 7, t + 5000u * 7 - 1) == DDE_LOCK_NEW_CYCLE && l.locked, "locked after 8");
    // 2 missing cycles: still locked
    unsigned cc = c + 10; uint32_t tt = t + 5000u * 10;
    CHECK(dde_lock_on_ref(&l, cc, tt) == DDE_LOCK_NEW_CYCLE && l.locked, "gap of 2 keeps lock");
    CHECK(!dde_lock_poll(&l, tt + 19000), "poll within 3 cycles ok");
    // 3 missing cycles: lost
    cc += 4; tt += 20000;
    dde_lock_on_ref(&l, cc, tt);
    CHECK(!l.locked && l.lock_losses == 1, "gap of 3 loses lock");
    for (int i = 1; i < 7; i++) dde_lock_on_ref(&l, (cc + i) & 63, tt + 5000u * i);
    CHECK(!l.locked, "re-arm needs 8");
    dde_lock_on_ref(&l, (cc + 7) & 63, tt + 5000u * 7);
    CHECK(l.locked, "re-armed after 8 good cycles");
    cc += 7; tt += 35000;
    // poll timeout
    CHECK(!dde_lock_poll(&l, tt + 20000), "poll at 4 cycles - not yet");
    CHECK(dde_lock_poll(&l, tt + 20001) && !l.locked && l.lock_losses == 2, "poll loses lock");
    // cycle counter jump
    dde_lock_reset(&l);
    for (int i = 0; i < 8; i++) dde_lock_on_ref(&l, (60 + i) & 63, 1000u + 5000u * i);
    CHECK(l.locked, "locked across 63 -> 0 wrap");
    CHECK(dde_lock_on_ref(&l, 30, 1000u + 5000u * 8) == DDE_LOCK_BAD && !l.locked, "cycle jump");
    // new wake after silence
    dde_lock_reset(&l);
    for (int i = 0; i < 8; i++) dde_lock_on_ref(&l, i, 1000u + 5000u * i);
    CHECK(dde_lock_on_ref(&l, 0, 1000u + 5000u * 8 + 2000000u) == DDE_LOCK_NEW_WAKE && !l.locked && l.wakes == 2,
          "silence > 1 s = new wake");
}

int main(int argc, char **argv)
{
    dde_core_init();
    // args: vector files; "strict:FILE" = every target frame must be rebuilt bit-exactly
    // from the tables (true for the ign-on/engine-off window the tables came from).
    for (int i = 1; i < argc; i++) {
        bool strict = strncmp(argv[i], "strict:", 7) == 0;
        test_vectors(strict ? argv[i] + 7 : argv[i], strict);
    }
    test_pack_args();
    test_schedule();
    test_lock();
    if (fails) { printf("test_dde_emu_core: %d FAILURES\n", fails); return 1; }
    printf("test_dde_emu_core: all OK\n");
    return 0;
}
