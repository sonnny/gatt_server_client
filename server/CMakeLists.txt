
cmake_minimum_required(VERSION 3.12)
set(PICO_SDK_PATH "/home/sonny/pico/pico-sdk")
set(PICOTOOL_FETCH_FROM_GIT_PATH "/home/sonny/pico/picotool")
set(PICO_BOARD "pico_w")
include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
project(pico_counter C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(pico_counter
  main.c
  )
target_link_libraries(pico_counter
  pico_stdlib
  pico_multicore
  pico_btstack_ble
  pico_btstack_cyw43
  pico_cyw43_arch_none)
add_compile_options(-O2)
pico_enable_stdio_uart(pico_counter 1)
# add new directories here for your own use
target_include_directories(pico_counter 
  PUBLIC
    thread
    ble)
pico_btstack_make_gatt_header(pico_counter PRIVATE "${CMAKE_CURRENT_LIST_DIR}/ble/pico_demo_gatt_service.gatt")
pico_add_extra_outputs(pico_counter)
