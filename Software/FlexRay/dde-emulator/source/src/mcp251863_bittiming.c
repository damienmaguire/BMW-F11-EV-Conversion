// Bit-timing calculation, kept free of SDK dependencies so it can be unit
// tested on the host (see tests/test_bittiming.c).
#include "mcp251863.h"

// Classic CAN nominal bit timing. Prefers BRP=1 (largest number of TQ, best
// resolution), sample point as close to 80 % as possible (CiA 601 / Microchip
// reference configs), SJW = TSEG2.
bool mcp251863_calc_bittiming(uint32_t osc_hz, uint32_t bitrate, mcp_bittiming_t *bt)
{
    if (!bt || bitrate == 0) return false;
    for (uint32_t brp = 1; brp <= 256; brp++) {
        if (osc_hz % (brp * bitrate)) continue;
        uint32_t tq = osc_hz / (brp * bitrate);
        // NBTCFG: TSEG1 1..256, TSEG2 1..128, SJW 1..128; need sync+tseg1+tseg2
        if (tq < 8 || tq > 1 + 256 + 128) continue;
        // sample point = (1 + tseg1) / tq; target 80 % rounded up to a TQ
        uint32_t sp_tq = (tq * 80 + 99) / 100;
        uint32_t tseg1 = sp_tq - 1;
        uint32_t tseg2 = tq - sp_tq;
        if (tseg2 < 2 || tseg2 > 128 || tseg1 < 1 || tseg1 > 256) continue;
        bt->brp = brp;
        bt->tseg1 = tseg1;
        bt->tseg2 = tseg2;
        bt->sjw = tseg2;
        bt->tq_per_bit = tq;
        bt->sample_point_permille = (1000 * sp_tq) / tq;
        return true;
    }
    return false;
}

// CAN FD data-phase timing (DBTCFG + TDC). Precomputed table rather than a
// search, so every supported combination is an explicit, reviewable entry.
//   16 MHz: ~75 % sample point (few TQ available at 4 Mbit/s).
//   40 MHz: Microchip reference values (MCP25xxFD API / AN), 80 % sample point.
// TDCO = (DTSEG1_reg + 1) * DBRP  (Microchip recommendation), TDC auto mode.
typedef struct { uint32_t osc, dbr; uint8_t brp, tseg1, tseg2, sjw; } dtab_t;
static const dtab_t dtab[] = {
    //  osc        dbitrate  BRP TSEG1 TSEG2 SJW   (real values, register = value - 1)
    {16000000u, 1000000u,  1,  11,   4,   4},  // 16 TQ, SP 75.0 %
    {16000000u, 2000000u,  1,   5,   2,   2},  //  8 TQ, SP 75.0 %
    {16000000u, 4000000u,  1,   2,   1,   1},  //  4 TQ, SP 75.0 % (marginal, coarse)
    {20000000u, 1000000u,  1,  15,   4,   4},  // 20 TQ, SP 80.0 %
    {20000000u, 2000000u,  1,   7,   2,   2},  // 10 TQ, SP 80.0 %
    {20000000u, 4000000u,  1,   3,   1,   1},  //  5 TQ, SP 80.0 %
    {40000000u, 1000000u,  1,  31,   8,   8},  // 40 TQ, SP 80.0 %  DBTCFG 0x001E0707 TDCO 31
    {40000000u, 2000000u,  1,  15,   4,   4},  // 20 TQ, SP 80.0 %  DBTCFG 0x000E0303 TDCO 15
    {40000000u, 4000000u,  1,   7,   2,   2},  // 10 TQ, SP 80.0 %  DBTCFG 0x00060101 TDCO 7
};

bool mcp251863_calc_data_bittiming(uint32_t osc_hz, uint32_t dbitrate, mcp_data_bittiming_t *d)
{
    if (!d) return false;
    for (unsigned i = 0; i < sizeof dtab / sizeof dtab[0]; i++) {
        if (dtab[i].osc != osc_hz || dtab[i].dbr != dbitrate) continue;
        d->brp = dtab[i].brp;
        d->tseg1 = dtab[i].tseg1;
        d->tseg2 = dtab[i].tseg2;
        d->sjw = dtab[i].sjw;
        d->tq_per_bit = 1u + d->tseg1 + d->tseg2;
        d->sample_point_permille = (1000u * (1u + d->tseg1)) / d->tq_per_bit;
        d->tdco = d->tseg1 * d->brp; // = (DTSEG1 register value + 1) * DBRP
        d->tdcmod = 2;                            // auto: measure TDCV, add TDCO
        d->dbtcfg = ((uint32_t)(d->brp - 1) << 24) | ((uint32_t)(d->tseg1 - 1) << 16) |
                    ((uint32_t)(d->tseg2 - 1) << 8) | (uint32_t)(d->sjw - 1);
        d->tdc = ((uint32_t)d->tdcmod << 16) | ((uint32_t)(d->tdco & 0x3F) << 8); // TDCV=0, SID11EN=0, EDGFLTEN=0
        return osc_hz == d->brp * d->tq_per_bit * dbitrate;
    }
    return false;
}

uint8_t mcp251863_len_to_dlc(uint8_t len)
{
    static const uint8_t fd_len[] = {12, 16, 20, 24, 32, 48, 64};
    if (len <= 8) return len;
    for (uint8_t i = 0; i < 7; i++) if (len <= fd_len[i]) return (uint8_t)(9 + i);
    return 15;
}

uint8_t mcp251863_dlc_to_len(uint8_t dlc)
{
    static const uint8_t l[16] = {0, 1, 2, 3, 4, 5, 6, 7, 8, 12, 16, 20, 24, 32, 48, 64};
    return l[dlc & 0xF];
}
