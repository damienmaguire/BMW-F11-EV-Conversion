#!/bin/bash
# v1h.2 firmware builds (4 UF2s). Usage: build-v1h2.sh [dir-suffix ...]
export PICO_SDK_PATH=/workspace/pico-sdk PATH=/home/box/.local/lib/python3.13/site-packages/cmake/data/bin:/home/box/.local/bin:$PATH
cfg(){ cmake -G Ninja -S /workspace/scratch/emu-v1f -B /workspace/$1 -DCMAKE_C_COMPILER=/workspace/toolchain/xpack-arm-none-eabi-gcc-14.2.1-1.1/bin/arm-none-eabi-gcc -DCMAKE_CXX_COMPILER=/workspace/toolchain/xpack-arm-none-eabi-gcc-14.2.1-1.1/bin/arm-none-eabi-g++ -Dpicotool_DIR=/workspace/host-picotool-install/picotool -Dpioasm_DIR=/workspace/host-pioasm-install/pioasm -DPICO_PLATFORM=rp2350-arm-s -DPICO_BOARD=pico2 -DCMAKE_BUILD_TYPE=Release -DPICO_FLEXRAY_DDE_EMU=ON -DPICO_FR_USB_FAST=ON -DEMU_GEAR=1 -DEMU_STANDALONE=$2 -DPICO_FLEXRAY_BOARD_V2=$3 -DEMU_CAN_TX=$3 $4 > /tmp/cfg_$1.log 2>&1 || { echo "cfg fail $1"; tail /tmp/cfg_$1.log; return 1; }; ninja -C /workspace/$1 > /tmp/build_$1.log 2>&1 || { echo "build fail $1"; grep -E "error|FAILED" /tmp/build_$1.log | head -20; return 1; }; grep -c "warning:" /tmp/build_$1.log | sed "s/^/$1 warnings: /"; }
cfg build-v1h2-v1 0 OFF
cfg build-v1h2-v2 0 ON
cfg build-v1h2-sa-v1 1 OFF
cfg build-v1h2-sa-v2 1 ON
