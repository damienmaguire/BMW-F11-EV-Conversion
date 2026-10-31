#ifndef FLEXRAY_PINS_H
#define FLEXRAY_PINS_H

#if PICO_FLEXRAY_BOARD_V2
// Pi_Flex_V2 carrier: same FR1/FR2/BGE/STBN pins, FR3/FR4 parked, no relays,
// LEDs on GP8/GP9, MCP251863 CAN on spi0 (GP16-21).
#include "board_pico_flexray_v2.h"
#else

// -- Streamer Pins --
#define BGE_PIN 2
#define STBN_PIN 3

#define LED_PIN 20
#define RELAY_FR_1_2 17
#define RELAY_FR_3_4 18

#define TXD_FR_1_PIN 28
#define TXEN_FR_1_PIN 27
#define RXD_FR_1_PIN 26

#define TXD_FR_2_PIN 4
#define TXEN_FR_2_PIN 5
#define RXD_FR_2_PIN 6

#define TXD_FR_3_PIN 10
#define TXEN_FR_3_PIN 9
#define RXD_FR_3_PIN 8

#define TXD_FR_4_PIN 16
#define TXEN_FR_4_PIN 22
#define RXD_FR_4_PIN 21

#endif // PICO_FLEXRAY_BOARD_V2
#endif // FLEXRAY_PINS_H
