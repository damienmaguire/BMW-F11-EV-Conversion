// v1h: writes a dde_engine_status_t (56 B) and a dde_emu_diag_t (64 B) with known values
// to stdout; run.sh parses them with tools/ts_capture/frts.py (C vs Python layout check).
#include <stdio.h>
#include <string.h>
#include "dde_emu.h"
int main(void)
{
    dde_engine_status_t e; memset(&e, 0, sizeof e);
    e.version = 2; e.mode = 1; e.phase = 2; e.stkl = 0xC; e.rpm = 780; e.starts = 3; e.seq = 99;
    e.profile = 2; e.profile_req = 1; e.cas_flags = 5; e.out_flags = 3; e.d_to_82_ms = 840; e.d_len_ms = 120;
    e.d_end_after_82_ms = -720; e.t_rpm_ms = 90; e.aborts = 7;
    fwrite(&e, 1, sizeof e, stdout);
    dde_emu_diag_t d; memset(&d, 0, sizeof d);
    d.version = 2; d.lm_slot = 0x28; d.lm_cycle = 33; d.lm_flags = 0x12; d.miss[1] = 343; d.late[3] = 4;
    d.lm_r_entry = -27; d.lm_r_final = 1234; d.lm_isr_late_us = 6; d.lm_sched_lead_us = 21; d.lm_sched_ref = 0x27;
    d.lm_hdr_us = 9; d.hdr_isr_max_us = 14; d.alarm_isr_max_us = 31; d.miss_resched_close = 300; d.miss_crc_near = 2;
    d.miss_preempt = 5; d.resched_guard_skips = 6; d.crc_fail = 487; d.no_cand = 70000; d.static_fired = 123456;
    d.hdr_over_10us = 8; d.resched_min_us = 40; d.snap_fail = 17; d.bg_max_us = 23;
    fwrite(&d, 1, sizeof d, stdout);
    return 0;
}
