# Generated Cmake Pico project file

cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release CACHE STRING "Build type" FORCE)
endif()

# Initialise pico_sdk from installed location
# (note this can come from environment, CMake cache etc)

# == DO NOT EDIT THE FOLLOWING LINES for the Raspberry Pi Pico VS Code Extension to work ==
if(WIN32)
    set(USERHOME $ENV{USERPROFILE})
else()
    set(USERHOME $ENV{HOME})
endif()
set(sdkVersion 2.2.0)
set(toolchainVersion 14_2_Rel1)
set(picotoolVersion 2.2.0-a4)
set(picoVscode ${USERHOME}/.pico-sdk/cmake/pico-vscode.cmake)
if (EXISTS ${picoVscode})
    include(${picoVscode})
endif()
# ====================================================================================
set(PICO_BOARD pico2 CACHE STRING "Board type")

# Pull in Raspberry Pi Pico SDK (must be before project)
include(pico_sdk_import.cmake)

project(pico_flexray C CXX ASM)

# Initialise the Raspberry Pi Pico SDK
pico_sdk_init()

# Add executable. Default name is the project name, version 0.1

add_executable(pico_flexray
     src/main.c
     src/flexray_frame.c
     src/panda_usb.c
     src/usb_descriptors.c
     src/flexray_bss_streamer.c
     src/flexray_blocker.c
     src/flexray_forwarder.c
     src/flexray_injector.c
     src/flexray_fifo.c
     src/flexray_ts.c
     )

# TS-CAPTURE (FIXED): hardware frame-start timestamps on the stock pass-through repeater.
option(PICO_FLEXRAY_TS_DISABLE "Do not start the hardware frame timestamp chain" OFF)
if (PICO_FLEXRAY_TS_DISABLE)
    target_compile_definitions(pico_flexray PRIVATE PICO_FLEXRAY_TS_DISABLE=1)
else()
    target_compile_definitions(pico_flexray PRIVATE PICO_FLEXRAY_TS_DISABLE=0)
endif()
# OFF = V1 board (relays GP17/18, LED GP20, FR3/FR4).
# ON  = Pi_Flex_V2 carrier (no relays, LEDs GP8/9, MCP251863 CAN on spi0, FR3/FR4 parked).
#       PT-CAN capture (500 kbit/s classic) -> USB CAN records, SavvyCAN bus 2.
#       RX ONLY: Listen Only controller mode, driver built without send functions
#       (except DDE emulator builds with EMU_CAN_TX=ON: PT-CAN TX enabled at runtime by the host).
option(PICO_FLEXRAY_BOARD_V2 "Build for the Pi_Flex_V2 carrier (MCP251863 PT-CAN RX-only + status LEDs)" OFF)
if (PICO_FLEXRAY_BOARD_V2)
    target_sources(pico_flexray PRIVATE
        src/mcp251863.c
        src/mcp251863_bittiming.c
        src/v2_cantest.c
        )
    target_compile_definitions(pico_flexray PRIVATE PICO_FLEXRAY_BOARD_V2=1
        V2_CAN_MODE=0 V2_CAN_TEST_TX=0 V2_CAN_FD_DATA_BITRATE=0u)
    # PT-CAN DDE/EGS emulation TX (v1e): DDE emulator builds only, opt-in at runtime
    # from the host (--can-tx). Boots Listen Only; see board_pico_flexray_v2.h.
    option(EMU_CAN_TX "DDE emulator V2 build: PT-CAN DDE/EGS frame TX, enabled at runtime by the host" ON)
    if (PICO_FLEXRAY_DDE_EMU AND EMU_CAN_TX)
        target_sources(pico_flexray PRIVATE src/ptcan_emu.c src/ptcan_emu_core.c)
        target_compile_definitions(pico_flexray PRIVATE EMU_CAN_TX=1)
    else()
        target_compile_definitions(pico_flexray PRIVATE EMU_CAN_TX=0 MCP251863_RX_ONLY=1)
    endif()
else()
    target_compile_definitions(pico_flexray PRIVATE PICO_FLEXRAY_BOARD_V2=0)
endif()
# Bigger USB vendor transfers + tud_task during notify bursts (run 4 USB loss fix, untested on HW)
option(PICO_FR_USB_FAST "1 KB vendor IN transfers, 16 KB TX buffer" OFF)
if (PICO_FR_USB_FAST)
    target_compile_definitions(pico_flexray PRIVATE PICO_FR_USB_FAST=1)
endif()

# DDE emulator build (see Software/FlexRay/dde-emulator/README.md). OFF = normal capture build.
option(PICO_FLEXRAY_DDE_EMU "Build the DDE emulator: transmit the DDE static frames on FR1" OFF)
set(EMU_SYNC_BITS 0 CACHE STRING "1 = keep the DDE sync/startup bits on 0x3D (UNSAFE, UNTESTED)")
set(EMU_DYN_5C 1 CACHE STRING "1 = also send the NM frame 0x5C (minislot 0, cycles 1 mod 16), KL15/NM-gated, 60 s hard ceiling")
set(EMU_TSS_BITS 7 CACHE STRING "Transmitted TSS length in bits (3..15)")
set(EMU_DYN_E6 1 CACHE STRING "1 = send dynamic 0xE6 L9 (DT_PT_2, every 200 cycles, cycle%8==2)")
set(EMU_DYN_FB 1 CACHE STRING "1 = send dynamic 0xFB L5/L7/L9 (cycle%8 5/6/3, every 1000/40/1000 cycles)")
set(EMU_DYN_E8 1 CACHE STRING "1 = send dynamic 0xE8 L9/L5 (cycle%8 3/0, every 200/1000 cycles, L5 after ign)")
set(EMU_PREIGN 1 CACHE STRING "1 = pre-ign content until 60 ms after the first 0x103")
set(EMU_142 1 CACHE STRING "1 = send dynamic 0x142 L31 (every 4 cycles, cycle%4==3, from wake)")
set(EMU_10A 1 CACHE STRING "1 = send dynamic 0x10A L5 (cycle%8==2, every 200 cycles, KL15 on only)")
set(EMU_68 1 CACHE STRING "1 = send dynamic 0x68 L3 (cycle%8==5, every 1000 cycles, from wake)")
set(EMU_KL15_F1 1 CACHE STRING "1 = KL15 from 0xF1 byte 3 (0x20 on / 0xE0 off), 0x103 fallback; 0 = 0x103 only")
if (PICO_FLEXRAY_DDE_EMU)
    target_sources(pico_flexray PRIVATE src/dde_emu.c src/dde_emu_core.c)
    pico_generate_pio_header(pico_flexray ${CMAKE_CURRENT_LIST_DIR}/src/dde_emu_tx.pio)
    target_compile_definitions(pico_flexray PRIVATE PICO_FLEXRAY_DDE_EMU=1
        EMU_SYNC_BITS=${EMU_SYNC_BITS} EMU_DYN_5C=${EMU_DYN_5C} EMU_TSS_BITS=${EMU_TSS_BITS}
        EMU_DYN_E6=${EMU_DYN_E6} EMU_DYN_FB=${EMU_DYN_FB} EMU_DYN_E8=${EMU_DYN_E8} EMU_PREIGN=${EMU_PREIGN}
        EMU_142=${EMU_142} EMU_10A=${EMU_10A} EMU_68=${EMU_68} EMU_KL15_F1=${EMU_KL15_F1})
    if (EMU_SYNC_BITS)
        message(WARNING "EMU_SYNC_BITS=1: the emulator will claim to be a sync/startup node. UNSAFE, UNTESTED.")
    endif()
else()
    target_compile_definitions(pico_flexray PRIVATE PICO_FLEXRAY_DDE_EMU=0)
endif()

pico_set_program_name(pico_flexray "pico_flexray")
pico_set_program_version(pico_flexray "0.1")

# no_flash means the target is to run from RAM
# pico_set_binary_type(pico_flexray no_flash)


# Generate PIO header
pico_generate_pio_header(pico_flexray 
    ${CMAKE_CURRENT_LIST_DIR}/src/flexray_bss_streamer.pio
    ${CMAKE_CURRENT_LIST_DIR}/src/flexray_forwarder_with_injector.pio
    ${CMAKE_CURRENT_LIST_DIR}/src/flexray_tsmark.pio
    )

# Modify the below lines to enable/disable output over UART/USB
pico_enable_stdio_uart(pico_flexray 1)
# The panda interface is a vendor-specific USB device, not a CDC (serial) device.
# So we don't enable stdio_usb, but we will link the tinyusb_device library directly.
# pico_enable_stdio_usb(pico_flexray 1)

# Add the standard library to the build
target_link_libraries(pico_flexray
        hardware_spi
        hardware_gpio
        pico_stdlib
        tinyusb_device
        pico_multicore
        hardware_dma
        hardware_pio
        hardware_interp
        hardware_pwm
        hardware_timer
        pico_unique_id
        )

# Add the standard include files to the build
target_include_directories(pico_flexray PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
        ${CMAKE_CURRENT_LIST_DIR}/src
)

# Enable strict compiler warnings for C sources and treat warnings as errors
target_compile_options(pico_flexray PRIVATE
        $<$<COMPILE_LANGUAGE:C>:-Wall -Wextra -Wstrict-prototypes -Werror>
)

# Add any user requested libraries
target_link_libraries(pico_flexray 
        )

pico_add_extra_outputs(pico_flexray)
