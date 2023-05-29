cmake_minimum_required(VERSION 3.13)
set(ENV{PICO_SDK_PATH} "~/pico/pico-sdk/")
include(pico_sdk_import.cmake)
project(sensorAcq C CXX ASM)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

# Tell the pico-sdk that we are compiling for a custom board
set(PICO_BOARD_HEADER_DIRS ${CMAKE_SOURCE_DIR})
set(PICO_BOARD openfc_sensAcq_rp2040)

pico_sdk_init()

add_executable(sensorAcq
  main.c
  usb_hal.c
  lsm6ds3.c
  timeSens.c
  sensAcqImpl.c
)
pico_add_extra_outputs(sensorAcq)

pico_enable_stdio_usb(sensorAcq 0)
pico_enable_stdio_uart(sensorAcq 0)
target_link_libraries(sensorAcq pico_stdlib hardware_timer hardware_resets hardware_irq hardware_spi hardware_dma)