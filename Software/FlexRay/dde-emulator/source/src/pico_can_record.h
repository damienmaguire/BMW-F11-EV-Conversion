// CAN record in the pico-flexray bulk-IN record stream (endpoint 0x81).
//
// The stream is a sequence of records, each [u16 LE length field][body]:
//   FlexRay format 1 : length = body length (< 0x8000), body[0] = source digits
//                      (1 = FR1 -> SavvyCAN bus 0, 2 = FR2 -> bus 1, 13/14/23/24 ...)
//   FlexRay format 2 : length = 0x8000 | body length, body[0] = 2 (version)
//   CAN record (this): length = 0x8000 | body length, body[0] = 0x43 ('C')
//
//   offset size  field
//     0     1    type   = PICO_CAN_REC_TYPE (0x43)
//     1     1    bus    = 2 on the V2 board (SavvyCAN bus number)
//     2     1    flags  bit0 IDE (29-bit ID), bit1 RTR, bit2 FDF, bit3 BRS,
//                       bit4 ESI, bit5 TX (v1f: a frame WE sent, from the MCP TX
//                       event FIFO; only if the host asked for TX records),
//                       bit7 data truncated (fewer bytes than DLC says)
//     3     1    dlc    raw DLC 0..15 as received
//     4     4    id     u32 LE, 11- or 29-bit identifier (no flag bits)
//     8     4    time   u32 LE, device time in us at SOF (RP2350 time_us_32
//                       clock, the same clock FlexRay format-2 records use;
//                       wraps every ~71.6 min)
//    12     n    data   n = body length - 12 bytes (0..8 classic, 0..64 FD;
//                       0 for remote frames)
//   body length = 12 + n  ->  14..22 bytes on the wire for classic CAN.
//
// Bit 15 of the length marks an "extended" record; body[0] says which kind.
// CAN records are only sent after the host opts in with the vendor control
// request PICOFLEX_SET_CAN_RECORDS (0xa1, wValue bit0 1 = on, 0 = off, OUT, no data;
// v1f: wValue bit1 = also send our own TX frames, flagged PICO_CAN_FLAG_TX),
// so hosts that don't know them (older SavvyCAN, flexray_stream_recorder.py)
// never receive one. The setting returns to off on USB unmount/reset.
#ifndef PICO_CAN_RECORD_H
#define PICO_CAN_RECORD_H

#include <stdint.h>
#include <stddef.h>
#include <string.h>

#define PICO_REC_LEN_EXTENDED      0x8000u
#define PICO_CAN_REC_TYPE          0x43u
#define PICO_CAN_REC_HDR_LEN       12u
#define PICO_CAN_REC_MAX_DATA      64u
#define PICO_CAN_REC_MAX_WIRE      (2u + PICO_CAN_REC_HDR_LEN + PICO_CAN_REC_MAX_DATA)
#define PICOFLEX_SET_CAN_RECORDS   0xa1u

#define PICO_CAN_FLAG_IDE        0x01u
#define PICO_CAN_FLAG_RTR        0x02u
#define PICO_CAN_FLAG_FDF        0x04u
#define PICO_CAN_FLAG_BRS        0x08u
#define PICO_CAN_FLAG_ESI        0x10u
#define PICO_CAN_FLAG_TX         0x20u
#define PICO_CAN_FLAG_TRUNCATED  0x80u

typedef struct {
    uint8_t bus;
    uint8_t flags;
    uint8_t dlc;
    uint8_t len;        // data bytes (<= PICO_CAN_REC_MAX_DATA)
    uint32_t id;
    uint32_t time_us;
    uint8_t data[PICO_CAN_REC_MAX_DATA];
} pico_can_record_t;

// Encodes r into out (>= PICO_CAN_REC_MAX_WIRE bytes). Returns bytes written.
static inline size_t pico_can_record_encode(const pico_can_record_t *r, uint8_t *out)
{
    uint8_t n = r->len > PICO_CAN_REC_MAX_DATA ? PICO_CAN_REC_MAX_DATA : r->len;
    uint16_t lenf = (uint16_t)(PICO_REC_LEN_EXTENDED | (PICO_CAN_REC_HDR_LEN + n));
    out[0] = (uint8_t)lenf;
    out[1] = (uint8_t)(lenf >> 8);
    out[2] = PICO_CAN_REC_TYPE;
    out[3] = r->bus;
    out[4] = r->flags;
    out[5] = r->dlc;
    for (int i = 0; i < 4; i++) {
        out[6 + i] = (uint8_t)(r->id >> (8 * i));
        out[10 + i] = (uint8_t)(r->time_us >> (8 * i));
    }
    memcpy(&out[14], r->data, n);
    return 14u + n;
}

#endif
