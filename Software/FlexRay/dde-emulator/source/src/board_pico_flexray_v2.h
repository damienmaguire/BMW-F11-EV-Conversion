// Pin map for the pico-flexray "Pi_Flex_V2" carrier board (Raspberry Pi Pico 2,
// RP2350A). Extracted from the KiCad V2 netlist (Pi_Flex_V2.kicad_sch).
//
//   FlexRay transceivers (NCV7383, IC1/IC3):
//     firmware FR1 = schematic channel 2 (IC3): TXD2=GP28 TXEN2=GP27 RXD2=GP26
//     firmware FR2 = schematic channel 1 (IC1): TXD1=GP4  TXEN1=GP5  RXD1=GP6
//     BGE (both) = GP2, STBN (both) = GP3
//   CAN FD controller + transceiver (MCP251863, IC4) on SPI0:
//     MISO(SDO)=GP16 CS=GP17 SCK=GP18 MOSI(SDI)=GP19 nINT=GP20 STBY=GP21
//     nINT0/nINT1/CLKO not connected. 16 MHz crystal (X1, X322516MLB4SI).
//   Status LEDs (active high, 470R to GND):
//     STAT1 (LED2 in schematic) = GP8, STAT2 (LED3 in schematic) = GP9
//   Test points: TP1 = GP7 (ISR profiling pin), TP4 = GP15.
//
// The V2 board has only two FlexRay transceivers and no relays. The main
// firmware's FR3/FR4 pins (GP8/9/10, GP16/21/22), relay pins (GP17/18) and
// LED pin (GP20) collide with the V2 LEDs and CAN SPI, so on V2 FR3/FR4 are
// parked on GPIOs that are not connected on the V2 board (see below).
#ifndef BOARD_PICO_FLEXRAY_V2_H
#define BOARD_PICO_FLEXRAY_V2_H

// -- FlexRay (unchanged vs main for FR1/FR2) --
#define BGE_PIN 2
#define STBN_PIN 3

#define TXD_FR_1_PIN 28
#define TXEN_FR_1_PIN 27
#define RXD_FR_1_PIN 26

#define TXD_FR_2_PIN 4
#define TXEN_FR_2_PIN 5
#define RXD_FR_2_PIN 6

// FR3/FR4 do not exist on V2. The PIO programs are still loaded exactly as on
// main (so timing/resource usage is identical), but on pins that are
// unconnected on the V2 PCB. RXD pins get pull-ups, so they idle recessive.
#define TXD_FR_3_PIN 10
#define TXEN_FR_3_PIN 11
#define RXD_FR_3_PIN 12

#define TXD_FR_4_PIN 13
#define TXEN_FR_4_PIN 14
#define RXD_FR_4_PIN 22

#define DEBUG_PROFILE_PIN 7 // TP1

// -- Status LEDs (active high) --
#define V2_LED1_PIN 8 // STAT1: heartbeat 1 Hz
#define V2_LED2_PIN 9 // STAT2: CAN TX status
#define V2_LED_ON 1
#define V2_LED_OFF 0
#define V2_PICO_ONBOARD_LED_PIN 25 // Pico 2 module LED, mirrors LED1

// -- MCP251863 CAN FD controller on SPI0 --
#define V2_CAN_SPI spi0
#define V2_CAN_SPI_MISO_PIN 16 // SPI0 RX
#define V2_CAN_SPI_CS_PIN 17   // SPI0 CSn (driven as plain GPIO)
#define V2_CAN_SPI_SCK_PIN 18  // SPI0 SCK
#define V2_CAN_SPI_MOSI_PIN 19 // SPI0 TX
#define V2_CAN_INT_PIN 20      // nINT, active low (push-pull by default)
#define V2_CAN_STBY_PIN 21     // transceiver STBY, high = standby
// SPI clock: datasheet requires FSCK <= 0.85 * SYSCLK / 2 (6.8 MHz at 16 MHz).
#define V2_CAN_SPI_HZ 4000000u

// Oscillator fitted on the controller (X1). Datasheet only specifies 4/20/40
// MHz crystals; the V2 BOM has 16 MHz. Change here if X1 is replaced.
#ifndef V2_CAN_OSC_HZ
#define V2_CAN_OSC_HZ 16000000u
#endif

#define V2_CAN_NOMINAL_BITRATE 500000u

// ---- Controller mode -------------------------------------------------------
// V2_CAN_MODE_LISTEN_ONLY (default): REQOP "Listen Only". The controller never
//   ACKs, never sends error frames and never transmits - safe on a car bus.
//   (A lone sender on a bench gets no ACK from us and keeps retrying.)
// V2_CAN_MODE_NORMAL: normal CAN 2.0 (or CAN FD if V2_CAN_FD_DATA_BITRATE != 0).
//   ACKs every frame and takes part in error signalling - bench use, or when
//   this board must be the ACKing node. Required for V2_CAN_TEST_TX.
#define V2_CAN_MODE_LISTEN_ONLY 0
#define V2_CAN_MODE_NORMAL      1
#ifndef V2_CAN_MODE
#define V2_CAN_MODE V2_CAN_MODE_LISTEN_ONLY
#endif

// CAN FD data-phase bit rate (BRS): 0 = classic CAN 2.0 (default for the RX
// build). 1000000u / 2000000u / 4000000u select the precomputed FD timings in
// mcp251863_bittiming.c (16, 20 and 40 MHz oscillators). With FD on, the RX
// FIFO switches to 64-byte objects (16 deep) and FD records carry FDF/BRS.
#ifndef V2_CAN_FD_DATA_BITRATE
#define V2_CAN_FD_DATA_BITRATE 0u
#endif

// ---- Test TX (off by default) ------------------------------------------------
// 1 = send 0x123 classic DLC 8 every 100 ms (byte0 counter, 12 34 56 78 9A BC DE)
//     and, with FD on, 0x124 FD+BRS 64 bytes 50 ms later. Needs V2_CAN_MODE_NORMAL.
#ifndef V2_CAN_TEST_TX
#define V2_CAN_TEST_TX 0
#endif
#if V2_CAN_TEST_TX && (V2_CAN_MODE != V2_CAN_MODE_NORMAL)
#error "V2_CAN_TEST_TX needs V2_CAN_MODE V2_CAN_MODE_NORMAL (Listen Only cannot transmit)"
#endif
// ---- PT-CAN DDE/EGS emulation TX (v1e, DDE emulator V2 build only) -----------
// EMU_CAN_TX=1: the controller still BOOTS in Listen Only (never ACKs, never sends
// error frames, never transmits). Only when the host enables CAN TX (vendor request
// 0xa5, logger --can-tx / --can-tx-ids) does ptcan_emu.c switch it to Normal CAN 2.0
// (via Configuration mode); it goes back to Listen Only on disable, keepalive timeout,
// bus-off, error passive or a foreign sender of an emulated ID. TX itself is gated on
// PT-CAN traffic (ptcan_emu_core.c). EMU_CAN_TX=0: strictly RX-only as in v1d.
#ifndef EMU_CAN_TX
#define EMU_CAN_TX 0
#endif
#if EMU_CAN_TX && !PICO_FLEXRAY_DDE_EMU
#error "EMU_CAN_TX is part of the DDE emulator build"
#endif
#if EMU_CAN_TX && defined(MCP251863_RX_ONLY)
#error "EMU_CAN_TX needs the driver send functions (do not define MCP251863_RX_ONLY)"
#endif
#define V2_CAN_TX_FIFO_DEPTH (EMU_CAN_TX ? 16 : 1)
#if PICO_FLEXRAY_DDE_EMU || defined(MCP251863_RX_ONLY)
// DDE emulator V2 build: Listen Only boot mode (never ACKs, never sends error
// frames), no test TX, classic CAN. Without EMU_CAN_TX the driver is compiled
// without its send functions (MCP251863_RX_ONLY).
#if !EMU_CAN_TX && !defined(MCP251863_RX_ONLY)
#error "DDE emulator V2 build without EMU_CAN_TX must define MCP251863_RX_ONLY"
#endif
#if V2_CAN_MODE != V2_CAN_MODE_LISTEN_ONLY
#error "DDE emulator V2 build: V2_CAN_MODE must be V2_CAN_MODE_LISTEN_ONLY (no CAN TX)"
#endif
#if V2_CAN_TEST_TX
#error "DDE emulator V2 build: V2_CAN_TEST_TX must be 0 (no CAN TX)"
#endif
#if V2_CAN_FD_DATA_BITRATE != 0
#error "DDE emulator V2 build: PT-CAN is classic 500 kbit/s"
#endif
#endif
#define V2_CAN_TEST_ID 0x123u
#define V2_CAN_TEST_PERIOD_MS 100
#define V2_CAN_FD_TEST_ID 0x124u
#define V2_CAN_FD_TEST_LEN 64
#define V2_CAN_TICK_MS (V2_CAN_TEST_PERIOD_MS / 2) // 50 ms housekeeping tick

// ---- RX / USB ------------------------------------------------------------------
#define V2_CAN_USB_BUS 2             // SavvyCAN bus number of this CAN channel
#define V2_CAN_RX_MAX_PER_POLL 8     // frames drained per main-loop pass (~65 us each)
#define V2_CAN_OVERFLOW_LED_MS 1000  // LED2 solid this long after an RX overflow
// 1 = put CAN records in the bulk-IN stream without the host opting in (only
// for hosts that know the record; older parsers resync byte by byte over it).
#ifndef V2_CAN_RECORDS_DEFAULT_ON
#define V2_CAN_RECORDS_DEFAULT_ON 0
#endif

#endif
