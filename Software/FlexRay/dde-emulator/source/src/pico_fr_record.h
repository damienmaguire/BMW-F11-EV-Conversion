// FlexRay records in the pico-flexray bulk-IN record stream (endpoint 0x81),
// selected by the host with vendor request PICOFLEX_SET_RECORD_FORMAT (0xa0,
// OUT, no data, wValue = format). The format returns to 1 on USB unmount.
//
// Every record is [u16 LE length field][body].
//
//   format 1 (default, stock): length = body length (< 0x8000)
//     body: [u8 source][5-byte header][payload][3-byte frame CRC, big endian]
//
//   format 2 (SavvyCAN picocan asks for this one automatically):
//     length = 0x8000 | body length
//     body: [u8 version = 2][u8 flags][u8 source][u32 LE device time, us]
//           [5-byte header][payload][3-byte CRC]
//
//   format 3 (high-resolution timestamps, tools/ts_capture/fr_ts_logger.py):
//     length = 0x8000 | body length
//     body: off size
//            0   1   version = 3
//            1   1   flags (below)
//            2   1   source (1 = FR1, 2 = FR2, decimal digits as format 1)
//            3   4   t_us   u32 LE: TIMER0 (time_us_32) latched by DMA at frame start
//            7   2   t_fine u16 LE: clk_sys counter (PWM slice 11) latched at frame start
//            9   4   seq    u32 LE: +1 for every CRC-valid frame the capture accepted
//                    after arming, counted BEFORE the USB FIFO -> a gap = frames
//                    dropped because the USB FIFO was full
//           13   5   FlexRay header, then payload, then 3-byte frame CRC (big endian)
//
//   flags (formats 2 and 3): bit0 frame CRC ok, bit1 header CRC ok,
//                            bit2 hardware timestamp valid (else t_us = CPU time at parse,
//                                 t_fine = 0)
//
// t_us and t_fine of one record were latched ~40-60 ns apart by the same DMA
// chain. clk_sys (100 MHz) and the 1 MHz tick both come from the 12 MHz crystal,
// so the host can rebuild a 10 ns resolution time (see fr_ts_logger.py):
//   d = (t_fine - t_us * R) mod 65536, R = fine_hz / 1e6 (= 100)
//   d0 = d of the first record;  t = unwrap(t_us) + wrap16(d - d0) / R   [us]
//
// CAN records (V2 board, type 0x43, vendor request 0xa1) are in pico_can_record.h.
#ifndef PICO_FR_RECORD_H
#define PICO_FR_RECORD_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define PICOFLEX_SET_RECORD_FORMAT 0xa0u
#define PICOFLEX_GET_TS_INFO       0xa2u

#define PICO_FR_FORMAT_STOCK  1u
#define PICO_FR_FORMAT_US     2u
#define PICO_FR_FORMAT_HIRES  3u

#define PICO_FR_REC_LEN_EXTENDED 0x8000u
#define PICO_FR_FLAG_FRAME_CRC_OK  0x01u
#define PICO_FR_FLAG_HEADER_CRC_OK 0x02u
#define PICO_FR_FLAG_TS_HW         0x04u

#define PICO_FR_V2_PREFIX 7u   // version, flags, source, u32 us
#define PICO_FR_V3_PREFIX 13u  // version, flags, source, u32 us, u16 fine, u32 seq
#define PICO_FR_MAX_PAYLOAD 254u
#define PICO_FR_MAX_WIRE (2u + PICO_FR_V3_PREFIX + 5u + PICO_FR_MAX_PAYLOAD + 3u)

typedef struct {
    uint8_t source;        // decimal digits (1, 2, ...)
    uint8_t flags;
    uint8_t header[5];
    const uint8_t *payload;
    uint16_t payload_len;  // bytes, = 2 * header payload-length words
    uint32_t crc24;
    uint32_t t_us;
    uint16_t t_fine;
    uint32_t seq;
} pico_fr_record_t;

static inline void pico_fr_put_le32(uint8_t *p, uint32_t v)
{
    p[0] = (uint8_t)v; p[1] = (uint8_t)(v >> 8); p[2] = (uint8_t)(v >> 16); p[3] = (uint8_t)(v >> 24);
}

// Encodes r in the given format into out (>= PICO_FR_MAX_WIRE bytes).
// Returns the number of bytes written (0 for an unknown format).
static inline size_t pico_fr_record_encode(uint8_t format, const pico_fr_record_t *r, uint8_t *out)
{
    size_t prefix;
    if (format == PICO_FR_FORMAT_STOCK) prefix = 1u;
    else if (format == PICO_FR_FORMAT_US) prefix = PICO_FR_V2_PREFIX;
    else if (format == PICO_FR_FORMAT_HIRES) prefix = PICO_FR_V3_PREFIX;
    else return 0;

    uint16_t n = r->payload_len > PICO_FR_MAX_PAYLOAD ? PICO_FR_MAX_PAYLOAD : r->payload_len;
    uint16_t body = (uint16_t)(prefix + 5u + n + 3u);
    uint16_t lenf = (format == PICO_FR_FORMAT_STOCK) ? body : (uint16_t)(PICO_FR_REC_LEN_EXTENDED | body);
    uint8_t *b = out + 2;
    out[0] = (uint8_t)lenf;
    out[1] = (uint8_t)(lenf >> 8);
    if (format == PICO_FR_FORMAT_STOCK) {
        b[0] = r->source;
    } else {
        b[0] = format;
        b[1] = r->flags;
        b[2] = r->source;
        pico_fr_put_le32(&b[3], r->t_us);
        if (format == PICO_FR_FORMAT_HIRES) {
            b[7] = (uint8_t)r->t_fine;
            b[8] = (uint8_t)(r->t_fine >> 8);
            pico_fr_put_le32(&b[9], r->seq);
        }
    }
    memcpy(&b[prefix], r->header, 5);
    if (n) memcpy(&b[prefix + 5u], r->payload, n);
    b[prefix + 5u + n] = (uint8_t)(r->crc24 >> 16);
    b[prefix + 6u + n] = (uint8_t)(r->crc24 >> 8);
    b[prefix + 7u + n] = (uint8_t)r->crc24;
    return 2u + body;
}

// Reply to PICOFLEX_GET_TS_INFO (0xa2, vendor IN, wLength >= 64). All LE.
#define PICO_TS_INFO_VERSION 1u
typedef struct __attribute__((packed)) {
    uint8_t  info_version;     // 1
    uint8_t  record_format;    // current format (1/2/3)
    uint8_t  fine_bits;        // 16
    uint8_t  armed;            // capture arm gate passed
    uint32_t fine_hz;          // t_fine counter rate (clk_sys, 100000000)
    uint32_t now_us;           // time_us_32() when the reply was built
    uint32_t seq_next;         // next seq value (= valid frames accepted after arming)
    uint32_t usb_fifo_dropped; // frames lost because the 256-frame USB FIFO was full
    uint32_t notify_dropped;   // frame-end notifications lost (core1 -> core0 queue full)
    uint32_t ring_overflow;    // byte-ring slices too long (main loop fell > 264 B behind)
    uint32_t len_mismatch;     // impossible length fields while walking a slice
    uint32_t parse_fail;       // slice parse failures (resync by 1 byte); NOT CRC failures, see crc_fail
    uint32_t ts_matched;       // frames with a hardware timestamp
    uint32_t ts_missing;       // valid frames without one (t_us falls back to CPU time)
    uint32_t ts_skipped;       // ts entries with no valid frame (normal: CRC fails / echo-side marks)
    uint32_t held_pre_arm;     // valid frames before the arm gate opened (never streamed)
    uint32_t valid_frames;     // CRC-valid frames seen in total
    uint32_t crc_fail;         // decoded frames rejected by header/frame CRC (dropped before seq, no gap)
    uint32_t reserved[1];
} pico_ts_info_t;
_Static_assert(sizeof(pico_ts_info_t) == 64, "pico_ts_info_t is 64 bytes");

#endif
