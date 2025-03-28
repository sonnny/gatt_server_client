cmake_minimum_required(VERSION 3.12)
set(PICO_SDK_PATH "/home/sonny/pico/pico-sdk")
set(PICOTOOL_FETCH_FROM_GIT_PATH "/home/sonny/pico/picotool")
set(PICO_BOARD "pico_w")
include("${PICO_SDK_PATH}/external/pico_sdk_import.cmake")
project(GATT_Client C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
pico_sdk_init()
add_executable(GATT_Client
    client.c
    )
target_link_libraries(GATT_Client
    pico_stdlib
    pico_btstack_ble
    pico_btstack_cyw43
    pico_cyw43_arch_none
    hardware_sync
    )
pico_enable_stdio_uart(GATT_Client 1)
target_include_directories(GATT_Client PRIVATE
    ${CMAKE_CURRENT_LIST_DIR} # For btstack config
    )
target_compile_definitions(GATT_Client PRIVATE
    RUNNING_AS_CLIENT=1
)
pico_add_extra_outputs(GATT_Client)
#pico_btstack_make_gatt_header(pico_counter PRIVATE "${CMAKE_CURRENT_LIST_DIR}/ble/pico_demo_gatt_service.gatt")
#pico_add_extra_outputs(pico_counter)
