// bluetooth server demo
// minimum files
// file broken down to pieces to easily organize and add functions
// if you create another directory for your own functions, update CMakeLists.txt
// command ON, OFF toggle picow led
// ON13, OFF13 toggle gpio13

#include <stdio.h>
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "thread/pt_cornell_rp2040_v1_3.h"
#include "ble/ble.h"
#include "pico/cyw43_arch.h"

char ble_data[80]; // this is declared extern in ble.h and copy new ble data to this variable

static PT_THREAD (ble_thread(struct pt *pt)){
  PT_BEGIN(pt);
  gpio_init(13);
  gpio_set_dir(13, GPIO_OUT);
  while(1){
    PT_SEM_SAFE_WAIT(pt, &BLUETOOTH_READY); // wait until a new data received from bluetooth
    if (strcmp(ble_data, "ON") == 0) cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    else if (strcmp(ble_data, "OFF") == 0) cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    else if (strcmp(ble_data, "ON13") == 0) gpio_put(13, 1);
    else if (strcmp(ble_data, "OFF13") == 0) gpio_put(13, 0);
    //printf("data from ble: %s\n", ble_data);
  }
  PT_END(pt);}
  
static PT_THREAD (blink_thread(struct pt *pt)){
  PT_BEGIN(pt);
  static bool led_state = false;
  gpio_init(15);
  gpio_set_dir(15, GPIO_OUT);
  PT_INTERVAL_INIT();
  while(1){
    led_state = !led_state;
    gpio_put(15, led_state);
    PT_YIELD_INTERVAL(1000000);}
  PT_END(pt);}

void main(void){

stdio_init_all();
sleep_ms(500);
multicore_launch_core1(bt_main);
pt_add_thread(ble_thread);
pt_add_thread(blink_thread);
pt_sched_method = SCHED_ROUND_ROBIN;
pt_schedule_start;}

