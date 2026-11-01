// v1h.1: dump a dde_gear_status_t (USB 0xab) and the C event texts, so run.sh can check
// that tools/ts_capture/frts.py parses the same layout and prints the same "GEAR: ..." texts.
#include <stdio.h>
#include <string.h>
#include "dde_gear.h"

static gear_t g;
static uint8_t crc8(const uint8_t *b, unsigned n)
{
    uint8_t c = 0;
    for (unsigned i = 0; i < n; i++) { c ^= b[i]; for (int k = 0; k < 8; k++) c = (uint8_t)((c & 0x80) ? (c << 1) ^ 0x1D : c << 1); }
    return c;
}
static void lever(uint32_t t, uint8_t pos, uint8_t pb)
{
    static uint8_t a;
    uint8_t d[4] = {0, (uint8_t)(0xF0 | a), (uint8_t)(pos << 4 | 0xF), pb}, b[5] = {0x3F, 0, d[1], d[2], d[3]};
    a = (uint8_t)((a + 1) % 15);
    d[0] = crc8(b, 5);
    gear_input_lever(&g, d, 4, t);
}
static uint32_t T = 1000000;
static void run(unsigned ms, uint8_t pos, uint8_t pb, bool brake)
{
    for (unsigned i = 0; i < ms; i += 5) {
        T += 5000;
        if (i % 20 == 0) { lever(T, pos, pb); if (brake) gear_input_brake(&g, T); }
        gear_step(&g, T, true);
    }
}

int main(int argc, char **argv)
{
    (void)argc;
    gear_reset(&g, true);
    run(200, 0, 0xC0, false);
    run(200, LV_FWD2, 0xC0, false);   // refused, no brake
    run(200, 0, 0xC0, true);
    run(200, LV_FWD2, 0xC0, true);    // P -> R
    run(300, 0, 0xC0, true);
    run(200, LV_BACK2, 0xC0, true);   // R -> D
    run(300, 0, 0xC0, true);
    run(100, 0, 0xD5, true);          // P button
    run(1000, 0, 0xC0, true);         // P shown, pawl not yet (brake held)
    { uint8_t bad[4] = {0}; gear_input_lever(&g, bad, 4, T); }   // a bad CRC
    dde_gear_status_t st;
    gear_fill_status(&g, &st);
    fwrite(&st, sizeof st, 1, stdout);
    FILE *f = fopen(argv[1], "w");
    for (unsigned i = 0; i < GEAR_N_EV; i++) { char b[96]; gear_ev_text(&st.ev[i], b, sizeof b); fprintf(f, "%s\n", b); }
    fclose(f);
    return 0;
}
