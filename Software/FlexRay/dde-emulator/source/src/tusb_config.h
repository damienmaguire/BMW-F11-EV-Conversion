#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

//--------------------------------------------------------------------
// COMMON CONFIGURATION
//--------------------------------------------------------------------
#define CFG_TUD_ENABLED         (1)

// Legacy RHPORT configuration
#define CFG_TUSB_RHPORT0_MODE   (OPT_MODE_DEVICE | OPT_MODE_FULL_SPEED)
#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT        (0)
#endif

#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU OPT_MCU_RP2040
#endif

#define CFG_TUSB_OS OPT_OS_PICO

#ifndef CFG_TUD_ENDPOINT0_SIZE
#define CFG_TUD_ENDPOINT0_SIZE  (64)
#endif

//--------------------------------------------------------------------
// DEVICE CONFIGURATION
//--------------------------------------------------------------------

// Vendor class for Panda protocol
#define CFG_TUD_VENDOR            1
#define CFG_TUD_VENDOR_RX_BUFSIZE 4096
#ifndef PICO_FR_USB_FAST
#define PICO_FR_USB_FAST 0
#endif
#if PICO_FR_USB_FAST
// TS-CAPTURE USB throughput: the vendor stream moves at most EPSIZE bytes per
// transfer and is only re-armed from tud_task(). With 64 B the IN rate was
// capped at ~217 KB/s (run 4, 4.5 % frames dropped). 1024 B transfers are
// split into 64 B packets by the DCD in the ISR. The descriptor wMaxPacketSize
// stays 64. Caveat: a bulk-OUT write of exactly N*64 bytes now completes only
// on the next short packet/ZLP (the ts logger sends no bulk OUT).
#define CFG_TUD_VENDOR_EPSIZE     1024
#define CFG_TUD_VENDOR_TX_BUFSIZE 16384
#else
#define CFG_TUD_VENDOR_TX_BUFSIZE 8192
#endif

// Enable debug for troubleshooting (only if not already defined)
#ifndef CFG_TUSB_DEBUG
#define CFG_TUSB_DEBUG            0
#endif

// Disable other classes
#define CFG_TUD_CDC               0
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_AUDIO             0
#define CFG_TUD_VIDEO             0
#define CFG_TUD_DFU_RUNTIME       0
#define CFG_TUD_DFU               0
#define CFG_TUD_ECM_RNDIS         0
#define CFG_TUD_NCM               0

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */ 