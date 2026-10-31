// V2 board extras: status LEDs and the MCP251863 CAN channel (RX to USB as
// SavvyCAN bus 2; optional test TX behind V2_CAN_TEST_TX).
#ifndef V2_CANTEST_H
#define V2_CANTEST_H
#include <stdbool.h>
#include <stdint.h>

struct can_health_t;

void v2_leds_init(void);
// Initialise the CAN controller (blocking, < ~30 ms). Call on core0 before the
// FlexRay cores are started. Prints the result over stdio.
void v2_can_init(void);
// Start the 50 ms hardware-alarm tick (LED1 heartbeat, LED2, status, test TX).
void v2_tick_start(void);
// Non-blocking service routine for the core0 main loop: drains at most
// V2_CAN_RX_MAX_PER_POLL frames (~65 us SPI each) while nINT is low, then the
// 50 ms housekeeping when due. Never waits on USB.
void v2_cantest_poll(void);
void v2_cantest_print_status(void);
// Fills the panda CAN health struct from cached values (no SPI access).
void v2_can_fill_health(struct can_health_t *h);
#endif
