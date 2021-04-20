# CMAKE generated file: DO NOT EDIT!
# Generated by "NMake Makefiles" Generator, CMake Version 3.19

# compile ASM with E:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2020-q4-major/bin/arm-none-eabi-gcc.exe
# compile C with e:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2020-q4-major/bin/arm-none-eabi-gcc.exe
# compile CXX with e:/Program Files (x86)/GNU Arm Embedded Toolchain/10 2020-q4-major/bin/arm-none-eabi-g++.exe
ASM_DEFINES = -DCFG_TUSB_DEBUG=1 -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DPICO_BIT_OPS_PICO=1 -DPICO_BOARD=\"pico\" -DPICO_BUILD=1 -DPICO_BUILD_BOOT_STAGE2_NAME=\"boot2_w25q080\" -DPICO_CMAKE_BUILD_TYPE=\"Debug\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_DIVIDER_HARDWARE=1 -DPICO_DOUBLE_PICO=1 -DPICO_FLOAT_PICO=1 -DPICO_INT64_OPS_PICO=1 -DPICO_MEM_OPS_PICO=1 -DPICO_MULTICORE=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_PRINTF_PICO=1 -DPICO_STDIO_USB=1 -DPICO_TARGET_NAME=\"i2c_slave_test\" -DPICO_USE_BLOCKED_RAM=0

ASM_INCLUDES = -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_S~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA268C~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_B~1\include -IP:\DOCUME~1\RASPIP~1\I2CSLA~1\build\GENERA~1\PICO_B~1 -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\boards\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_P~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\rp2040\HARDWA~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HARDWA~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\rp2040\HARDWA~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HARDWA~3\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA2A15~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HAE181~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA23F6~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_T~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA775E~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_S~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_U~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_R~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HARDWA~4\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA2362~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA33F0~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA78F3~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HAA248~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA3ACB~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA5FF7~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_P~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_B~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_B~3\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_D~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_D~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_I~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_F~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_M~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\BOOT_S~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_B~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_S~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PI24F1~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\lib\tinyusb\src -IP:\DOCUME~1\RASPIP~1\pico-sdk\lib\tinyusb\src\common -IP:\DOCUME~1\RASPIP~1\pico-sdk\lib\tinyusb\hw -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\pico_fix\RP2040~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA15F1~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_M~3\include

ASM_FLAGS = -march=armv6-m -mcpu=cortex-m0plus -mthumb -Og -g -ffunction-sections -fdata-sections

C_DEFINES = -DCFG_TUSB_DEBUG=1 -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DPICO_BIT_OPS_PICO=1 -DPICO_BOARD=\"pico\" -DPICO_BUILD=1 -DPICO_BUILD_BOOT_STAGE2_NAME=\"boot2_w25q080\" -DPICO_CMAKE_BUILD_TYPE=\"Debug\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_DIVIDER_HARDWARE=1 -DPICO_DOUBLE_PICO=1 -DPICO_FLOAT_PICO=1 -DPICO_INT64_OPS_PICO=1 -DPICO_MEM_OPS_PICO=1 -DPICO_MULTICORE=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_PRINTF_PICO=1 -DPICO_STDIO_USB=1 -DPICO_TARGET_NAME=\"i2c_slave_test\" -DPICO_USE_BLOCKED_RAM=0

C_INCLUDES = -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_S~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA268C~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_B~1\include -IP:\DOCUME~1\RASPIP~1\I2CSLA~1\build\GENERA~1\PICO_B~1 -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\boards\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_P~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\rp2040\HARDWA~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HARDWA~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\rp2040\HARDWA~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HARDWA~3\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA2A15~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HAE181~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA23F6~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_T~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA775E~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_S~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_U~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_R~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HARDWA~4\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA2362~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA33F0~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA78F3~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HAA248~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA3ACB~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA5FF7~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_P~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_B~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_B~3\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_D~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_D~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_I~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_F~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_M~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\BOOT_S~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_B~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_S~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PI24F1~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\lib\tinyusb\src -IP:\DOCUME~1\RASPIP~1\pico-sdk\lib\tinyusb\src\common -IP:\DOCUME~1\RASPIP~1\pico-sdk\lib\tinyusb\hw -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\pico_fix\RP2040~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA15F1~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_M~3\include

C_FLAGS = -march=armv6-m -mcpu=cortex-m0plus -mthumb -Og -g -ffunction-sections -fdata-sections -std=gnu11

CXX_DEFINES = -DCFG_TUSB_DEBUG=1 -DCFG_TUSB_MCU=OPT_MCU_RP2040 -DCFG_TUSB_OS=OPT_OS_PICO -DPICO_BIT_OPS_PICO=1 -DPICO_BOARD=\"pico\" -DPICO_BUILD=1 -DPICO_BUILD_BOOT_STAGE2_NAME=\"boot2_w25q080\" -DPICO_CMAKE_BUILD_TYPE=\"Debug\" -DPICO_COPY_TO_RAM=0 -DPICO_CXX_ENABLE_EXCEPTIONS=0 -DPICO_DIVIDER_HARDWARE=1 -DPICO_DOUBLE_PICO=1 -DPICO_FLOAT_PICO=1 -DPICO_INT64_OPS_PICO=1 -DPICO_MEM_OPS_PICO=1 -DPICO_MULTICORE=1 -DPICO_NO_FLASH=0 -DPICO_NO_HARDWARE=0 -DPICO_ON_DEVICE=1 -DPICO_PRINTF_PICO=1 -DPICO_STDIO_USB=1 -DPICO_TARGET_NAME=\"i2c_slave_test\" -DPICO_USE_BLOCKED_RAM=0

CXX_INCLUDES = -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_S~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA268C~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_B~1\include -IP:\DOCUME~1\RASPIP~1\I2CSLA~1\build\GENERA~1\PICO_B~1 -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\boards\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_P~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\rp2040\HARDWA~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HARDWA~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\rp2040\HARDWA~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HARDWA~3\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA2A15~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HAE181~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA23F6~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_T~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA775E~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_S~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_U~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_R~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HARDWA~4\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA2362~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA33F0~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA78F3~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HAA248~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA3ACB~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA5FF7~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_P~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_B~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_B~3\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_D~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_D~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_I~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_F~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_M~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\BOOT_S~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\common\PICO_B~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_S~2\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PI24F1~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\lib\tinyusb\src -IP:\DOCUME~1\RASPIP~1\pico-sdk\lib\tinyusb\src\common -IP:\DOCUME~1\RASPIP~1\pico-sdk\lib\tinyusb\hw -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\pico_fix\RP2040~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\HA15F1~1\include -IP:\DOCUME~1\RASPIP~1\pico-sdk\src\RP2_CO~1\PICO_M~3\include

CXX_FLAGS = -march=armv6-m -mcpu=cortex-m0plus -mthumb -Og -g -ffunction-sections -fdata-sections -fno-exceptions -fno-unwind-tables -fno-rtti -fno-use-cxa-atexit -std=gnu++17

