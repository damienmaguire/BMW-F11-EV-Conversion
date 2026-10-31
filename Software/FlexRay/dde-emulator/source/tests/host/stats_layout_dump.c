// Writes a dde_emu_stats_t with known values to stdout; run.sh parses it with
// tools/ts_capture/frts.py to check the Python v4 layout against the C struct.
#include <stdio.h>
#include <string.h>
#include "dde_emu.h"
int main(void){ dde_emu_stats_t s; memset(&s,0,sizeof s); s.version=4;
 s.dyn_sent2[0]=11; s.dyn_sent2[1]=22; s.dyn_sent2[2]=33; s.dyn_skip2[2]=7; s.dyn_last_ms2[0]=129;
 s.kl15=0x55; s.f1_b3=0xE0; s.kl15_on_edges=2; s.kl15_off_edges=3; s.nm_ceiling_stops=4;
 s.kl15_off_ago_ms=5000; s.ceil_left_ms=6000; s.isr_late_min_us=-2; s.isr_late_max_us=9; s.r_entry_min=-5; s.r_final_min=77;
 s.topup_timeouts=8; s.hk_deferred=65535; s.dyn_flags2=0x1B; s.ph142=1; s.abs_cycle=123456;
 fwrite(&s,1,sizeof s,stdout); return 0;}
