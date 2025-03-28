
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

// Pico headers
#include "btstack_run_loop_embedded.h"
#include "pico/stdlib.h"
#include "btstack_config.h"
#include "btstack_event.h"
#include "pico/cyw43_arch.h"
#include "pico/btstack_cyw43.h"
#include "pico/async_context.h"
#include "btstack.h"

// App headers
#include "pico_demo_gatt_service.h"
#include "gap_config.h"

extern char ble_data[80];
struct pt_sem BLUETOOTH_READY;

static int le_notification_enabled;
static btstack_packet_callback_registration_t hci_event_callback_registration;
static hci_con_handle_t con_handle;

#ifdef ENABLE_GATT_OVER_CLASSIC
static uint8_t gatt_service_buffer[70];
#endif

static int att_write_callback(hci_con_handle_t connection_handle, uint16_t att_handle, uint16_t transaction_mode, uint16_t offset, uint8_t *buffer, uint16_t buffer_size){
switch (att_handle){
  case ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_USER_DESCRIPTION_HANDLE:
    le_notification_enabled = little_endian_read_16(buffer, 0) == GATT_CLIENT_CHARACTERISTICS_CONFIGURATION_NOTIFICATION;
    con_handle = connection_handle;
    break;
  case ATT_CHARACTERISTIC_0000FF11_0000_1000_8000_00805F9B34FB_01_VALUE_HANDLE:
    buffer[buffer_size] = 0;
    //if (!strcmp(buffer, "OFF")) cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 0);
    //else if (!strcmp(buffer, "ON")) cyw43_arch_gpio_put(CYW43_WL_GPIO_LED_PIN, 1);
    memset(ble_data, 0, sizeof(ble_data));
    memcpy(ble_data, buffer, buffer_size);
    ble_data[buffer_size] = 0;
    PT_SEM_SAFE_SIGNAL(pt, &BLUETOOTH_READY); // signal main we have bluetooth data to process
    break;
  default:
    break;} return 0;}

void bt_main(void){
 if (cyw43_arch_init()) {
        printf("failed to initialise cyw43_arch\n");
        return -1;}
    hci_event_callback_registration.callback = &packet_handler;
    hci_add_event_handler(&hci_event_callback_registration);        
    l2cap_init();
    sm_init();
    att_server_init(profile_data, NULL, att_write_callback);
    att_server_register_packet_handler(packet_handler);
    printf("hci_power_control\n");
  hci_power_control(HCI_POWER_ON);
  printf("run loop\n");
  btstack_run_loop_execute();
  }
