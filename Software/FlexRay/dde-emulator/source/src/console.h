// Non-blocking debug console (v1f): printf() goes into a RAM ring, a 1 ms timer drains
// it into the UART FIFO. See console.c.
#ifndef CONSOLE_H
#define CONSOLE_H
#include <stdint.h>
void console_init(void);       // call right after stdio_init_all()
void console_drain(void);      // optional extra drain (main loop); never waits
uint32_t console_dropped(void); // characters dropped because the ring was full
uint32_t console_pending(void); // characters waiting
#endif
