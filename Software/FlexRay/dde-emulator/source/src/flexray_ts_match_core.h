// TS-CAPTURE: pure matching logic (no hardware), shared by flexray_ts.c and
// the host test (tests/host/test_ts_record.c).
#ifndef FLEXRAY_TS_MATCH_CORE_H
#define FLEXRAY_TS_MATCH_CORE_H
#include <stdbool.h>
#include <stdint.h>

typedef struct {
    uint32_t matched;      // frames that got a hardware timestamp
    uint32_t missing;      // valid frames with no matching timestamp entry
    uint32_t skipped;      // ts entries consumed without a frame (CRC fails, echo-side empty decodes, noise)
    uint32_t marks[2];     // ts entries consumed per side (FR1, FR2)
} flexray_ts_stats_t;

typedef struct {
    uint32_t us;    // TIMER0 (time_us_32) at frame start
    uint16_t fine;  // clk_sys counter mod 65536 at frame start
    bool valid;
} flexray_ts_t;

// Entries rd..w-1 (mod n_entries, a power of two) are unread. idx[] holds the
// captured data-DMA write address; (idx - ring_base) & ring_mask is the byte
// ring position of the frame start. Consumes everything up to and including
// the LAST entry whose position equals frame_start_idx; stops before the
// first entry that is newer (ahead of frame_start_idx by < half the ring).
static inline bool flexray_ts_match_core(const volatile uint32_t *us, const volatile uint32_t *fine,
                                         const volatile uint32_t *idx, uint32_t n_entries,
                                         uint32_t *rd, uint32_t w, uint32_t ring_base,
                                         uint16_t frame_start_idx, uint16_t ring_mask,
                                         flexray_ts_stats_t *st, int side, flexray_ts_t *out)
{
    out->valid = false;
    const uint16_t half = (uint16_t)((ring_mask + 1u) >> 1);
    while (*rd != w) {
        uint32_t i = *rd;
        uint16_t e = (uint16_t)((idx[i] - ring_base) & ring_mask);
        if (e == frame_start_idx) {
            if (out->valid) st->skipped++;   // earlier entry with the same position (echo-side empty decode)
            out->us = us[i];
            out->fine = (uint16_t)fine[i];
            out->valid = true;
            *rd = (i + 1u) & (n_entries - 1u);
            st->marks[side]++;
            continue;
        }
        if (out->valid) break;                         // next frame's entry: leave it
        uint16_t ahead = (uint16_t)((e - frame_start_idx) & ring_mask);
        if (ahead != 0 && ahead < half) break;          // newer than this frame: leave it
        *rd = (i + 1u) & (n_entries - 1u);              // older: no valid frame for it
        st->marks[side]++;
        st->skipped++;
    }
    if (out->valid) st->matched++;
    else st->missing++;
    return out->valid;
}
#endif
