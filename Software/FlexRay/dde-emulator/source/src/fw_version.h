// Firmware version string (v1h.2).
//
// In plain words: this is the name the firmware gives itself. It is printed on the
// debug console at power-up and the laptop logger asks for it over USB (the standard
// panda "git version" request, 0xd6). The logger prints both its own version and this
// one at the start and warns when they do not belong together (it checks the end of
// the text, "-v1h.2").
// Change it together with LOGGER_VERSION in tools/ts_capture/frts.py.
#ifndef FW_VERSION_H
#define FW_VERSION_H

#if PICO_FLEXRAY_DDE_EMU && EMU_STANDALONE
#define FW_VERSION "dde-emu-standalone-v1h.2"   // emulates from power-up, no laptop needed
#elif PICO_FLEXRAY_DDE_EMU
#define FW_VERSION "dde-emu-v1h.2"
#else
#define FW_VERSION "ts-capture-v1h.2"
#endif

#endif
