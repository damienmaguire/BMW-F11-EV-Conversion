// Host checks for the TS-CAPTURE record encoder (src/pico_fr_record.h) and the
// timestamp matching logic (src/flexray_ts_match_core.h).
// Usage: test_ts_record [stream.bin]  - optional: write a mixed format 1/2/3
// sample stream for tools/ts_capture/test_ts_tools.py to parse.
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "pico_fr_record.h"
#include "flexray_ts_match_core.h"

static int fails;
#define CHECK(c) do { if (!(c)) { printf("FAIL %s:%d: %s\n", __FILE__, __LINE__, #c); fails++; } } while (0)

static pico_fr_record_t sample(uint16_t id, uint8_t cycle, uint8_t nbytes, const uint8_t *pl)
{
    pico_fr_record_t r;
    memset(&r, 0, sizeof r);
    r.source = 2;
    r.flags = PICO_FR_FLAG_FRAME_CRC_OK | PICO_FR_FLAG_HEADER_CRC_OK | PICO_FR_FLAG_TS_HW;
    r.header[0] = (uint8_t)(0x00 | ((id >> 8) & 7));
    r.header[1] = (uint8_t)id;
    r.header[2] = (uint8_t)((nbytes / 2) << 1);
    r.header[3] = 0x5A;
    r.header[4] = (uint8_t)(0x40 | (cycle & 0x3F));
    r.payload = pl;
    r.payload_len = nbytes;
    r.crc24 = 0xABCDEF;
    r.t_us = 0xFFFFFF00u;
    r.t_fine = 0x1234;
    r.seq = 0x01020304;
    return r;
}

static void test_encoder(void)
{
    uint8_t pl[16];
    for (int i = 0; i < 16; i++) pl[i] = (uint8_t)(0xA0 + i);
    pico_fr_record_t r = sample(0x26, 5, 16, pl);
    uint8_t b[PICO_FR_MAX_WIRE];

    // format 1: byte-identical to the stock firmware's record
    size_t n = pico_fr_record_encode(1, &r, b);
    CHECK(n == 2 + 1 + 5 + 16 + 3);
    CHECK(b[0] == 25 && b[1] == 0);
    CHECK(b[2] == 2);
    CHECK(b[3] == 0x00 && b[4] == 0x26 && b[5] == 16 && b[6] == 0x5A && b[7] == 0x45);
    CHECK(b[8] == 0xA0 && b[23] == 0xAF);
    CHECK(b[24] == 0xAB && b[25] == 0xCD && b[26] == 0xEF);

    // format 2: SavvyCAN picocan layout
    n = pico_fr_record_encode(2, &r, b);
    CHECK(n == 2 + 7 + 5 + 16 + 3);
    CHECK((b[0] | (b[1] << 8)) == (0x8000 | 31));
    CHECK(b[2] == 2 && b[3] == 0x07 && b[4] == 2);
    CHECK(b[5] == 0x00 && b[6] == 0xFF && b[7] == 0xFF && b[8] == 0xFF);
    CHECK(b[9] == 0x00 && b[10] == 0x26);
    CHECK(b[14] == 0xA0 && b[30] == 0xAB && b[32] == 0xEF);

    // format 3: high resolution
    n = pico_fr_record_encode(3, &r, b);
    CHECK(n == 2 + 13 + 5 + 16 + 3);
    CHECK((b[0] | (b[1] << 8)) == (0x8000 | 37));
    CHECK(b[2] == 3 && b[3] == 0x07 && b[4] == 2);
    CHECK(b[5] == 0x00 && b[8] == 0xFF);
    CHECK(b[9] == 0x34 && b[10] == 0x12);
    CHECK(b[11] == 0x04 && b[12] == 0x03 && b[13] == 0x02 && b[14] == 0x01);
    CHECK(b[15] == 0x00 && b[16] == 0x26 && b[17] == 16 && b[19] == 0x45);
    CHECK(b[20] == 0xA0 && b[36] == 0xAB && b[38] == 0xEF);

    // null frame / zero payload, max payload, unknown format
    r.payload_len = 0;
    r.header[2] = 0;
    CHECK(pico_fr_record_encode(3, &r, b) == 2 + 13 + 5 + 3);
    CHECK(pico_fr_record_encode(0, &r, b) == 0);
    CHECK(pico_fr_record_encode(4, &r, b) == 0);
    static uint8_t big[254];
    r.payload = big;
    r.payload_len = 254;
    CHECK(pico_fr_record_encode(3, &r, b) == PICO_FR_MAX_WIRE);
    CHECK(sizeof(pico_ts_info_t) == 64);
}

// ---- matching -------------------------------------------------------------
#define N 256u
static uint32_t us_r[N], fine_r[N], idx_r[N];
static uint32_t w_, rd_;
static const uint32_t BASE = 0x20001000u;
static void mark(uint16_t pos, uint32_t us)
{
    us_r[w_] = us; fine_r[w_] = us * 100u; idx_r[w_] = BASE + pos;
    w_ = (w_ + 1u) & (N - 1u);
}
static bool match(uint16_t pos, flexray_ts_stats_t *st, flexray_ts_t *o)
{
    return flexray_ts_match_core(us_r, fine_r, idx_r, N, &rd_, w_, BASE, pos, 4095, st, 0, o);
}

static void test_match(void)
{
    flexray_ts_stats_t st = {0};
    flexray_ts_t o;
    w_ = rd_ = 0;

    // plain sequence: frames at 0, 30, 60
    mark(0, 100); mark(30, 200); mark(60, 300);
    CHECK(match(0, &st, &o) && o.us == 100);
    CHECK(match(30, &st, &o) && o.us == 200);
    CHECK(match(60, &st, &o) && o.us == 300);
    CHECK(st.matched == 3 && st.skipped == 0 && st.missing == 0);

    // echo-side empty decode leaves an entry at the same position; the real frame's (later) wins
    mark(90, 390); mark(90, 400);
    CHECK(match(90, &st, &o) && o.us == 400);
    CHECK(st.skipped == 1);

    // CRC-failed frame at 120 (never matched), next frame at 150 skips it
    mark(120, 500); mark(150, 600);
    CHECK(match(150, &st, &o) && o.us == 600);
    CHECK(st.skipped == 2);

    // a newer entry (frame not parsed yet) is left in place
    mark(180, 700); mark(210, 800);
    CHECK(match(180, &st, &o) && o.us == 700);
    CHECK(rd_ != w_);
    CHECK(match(210, &st, &o) && o.us == 800);

    // missing mark: frame parsed but no entry (e.g. tsmark FIFO full)
    CHECK(!match(240, &st, &o));
    CHECK(st.missing == 1);
    mark(270, 900);
    CHECK(match(270, &st, &o) && o.us == 900);

    // byte-ring wrap: frame at 4090 then at 20
    mark(4090, 1000); mark(20, 1100);
    CHECK(match(4090, &st, &o) && o.us == 1000);
    CHECK(match(20, &st, &o) && o.us == 1100);

    // ts ring lapped by echo-side marks at one position (long silence on that side)
    for (uint32_t k = 0; k < 300; k++) mark(50, 2000 + k);
    CHECK(match(50, &st, &o) && o.us == 2299);
    CHECK(rd_ == w_);
}

static void write_stream(const char *path)
{
    FILE *f = fopen(path, "wb");
    if (!f) { perror(path); fails++; return; }
    uint8_t pl[254];
    for (int i = 0; i < 254; i++) pl[i] = (uint8_t)i;
    uint8_t b[PICO_FR_MAX_WIRE];
    // 3 records per format: (id, cycle, len); t_us/t_fine/seq set per record
    const struct { uint16_t id; uint8_t cyc; uint8_t len; } recs[3] = {{0x26, 0, 16}, {0x5C, 1, 0}, {0x7FF, 63, 254}};
    for (int fmt = 1; fmt <= 3; fmt++)
        for (int k = 0; k < 3; k++) {
            pico_fr_record_t r = sample(recs[k].id, recs[k].cyc, recs[k].len, pl);
            r.source = (uint8_t)(1 + (k & 1));
            r.t_us = 1000u * (uint32_t)(fmt * 10 + k);
            r.t_fine = (uint16_t)(r.t_us * 100u + 37u);
            r.seq = (uint32_t)(fmt * 10 + k);
            size_t n = pico_fr_record_encode((uint8_t)fmt, &r, b);
            fwrite(b, 1, n, f);
        }
    fclose(f);
    printf("wrote %s\n", path);
}

int main(int argc, char **argv)
{
    test_encoder();
    test_match();
    if (argc > 1) write_stream(argv[1]);
    if (fails) { printf("%d FAILED\n", fails); return 1; }
    printf("test_ts_record: all checks passed\n");
    return 0;
}
