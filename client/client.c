/**
 * Copyright (c) 2023 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "btstack.h"
#include "pico/cyw43_arch.h"
#include "pico/stdlib.h"
#include "hardware/sync.h"
#include "pt_cornell_rp2040_v1_3_client.h"

// Show/conceal debugging information
//#if 1
#define DEBUG_LOG(...) printf(__VA_ARGS__)
//#else
//#define DEBUG_LOG(...)
//#endif

#define SERVICE_TO_SEARCH        0xFF10      // the only service were looking for to find the characteristic
#define CHARACTERISTIC_TO_SEARCH 0xFF11      // the only characteristic were looking for to write values
#define ADDRESS_TO_SEARCH        "28:CD:C1"  // picow mac address prefix, were only looking for picow

// UUID of custom service that we'll access
//#define CUSTOM_SERVICE 0xFF10
//static const uint8_t service_name[16] = {0x00, 0x00, 0xFF, 0x10, 0x00, 0x00, 0x10, 0x00, 0x80, 0x00, 0x00, 0x80, 0x5F, //0x9B, 0x34, 0xFB} ;

// UUID for the characteristic descriptors of interest
//#define CHARACTERISTIC_USER_DESCRIPTION 0x2901
//#define CHARACTERISTIC_CONFIGURATION    0x2902

// Bluetooth control structures
static btstack_packet_callback_registration_t hci_event_callback_registration;
static bd_addr_t server_addr;
static bd_addr_type_t server_addr_type;
static hci_con_handle_t connection_handle;
static gatt_client_service_t server_service;

static gatt_client_notification_t notification_listener;

// Protothreads semaphore
struct pt_sem characteristics_discovered ;

// Counts characteristics in ATT state machine
int k = 0 ;
// Counts characteristic descriptors in ATT state machine
int k2 = 0 ;
// Holds the number of discovered characteristics
int num_characteristics = 0 ;
// Holds the length of received packets in ATT state machine
uint32_t descriptor_length ;
//////////////////////////test variables

typedef struct advertising_report {
    uint8_t   type;
    uint8_t   event_type;
    uint8_t   address_type;
    bd_addr_t address;
    uint8_t   rssi;
    uint8_t   length;
    const uint8_t * data;
} advertising_report_t;

static bd_addr_t cmdline_addr;
static int cmdline_addr_found = 0;

static hci_con_handle_t connection_handle;
static gatt_client_service_t my_service;
static gatt_client_characteristic_t characteristic;
static bool service_found = false;
static bool show_characteristic_once = true;

static btstack_packet_callback_registration_t hci_event_callback_registration;

static bool found_device(advertising_report_t * e){
  if (strncmp(bd_addr_to_str(e->address), ADDRESS_TO_SEARCH, 8) == 0) return true;
  else return false;}
////////////////////////////////////////////
static void fill_advertising_report_from_packet(advertising_report_t * report, uint8_t *packet){
    gap_event_advertising_report_get_address(packet, report->address);
    report->event_type = gap_event_advertising_report_get_advertising_event_type(packet);
    report->address_type = gap_event_advertising_report_get_address_type(packet);
    report->rssi = gap_event_advertising_report_get_rssi(packet);
    report->length = gap_event_advertising_report_get_data_length(packet);
    report->data = gap_event_advertising_report_get_data(packet);
}

static void handle_gatt_client_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(packet_type);
    UNUSED(channel);
    UNUSED(size);

    gatt_client_service_t service;
    switch(hci_event_packet_get_type(packet)){
        case GATT_EVENT_SERVICE_QUERY_RESULT:
            gatt_event_service_query_result_get_service(packet, &service);
            if (service.uuid16 == SERVICE_TO_SEARCH) {
              my_service = service;
              service_found = true;}
  
            break;
        case GATT_EVENT_CHARACTERISTIC_QUERY_RESULT:
          if (show_characteristic_once) {
            gatt_event_characteristic_query_result_get_characteristic(packet, &characteristic);
           if (characteristic.uuid16 == CHARACTERISTIC_TO_SEARCH){
              printf("characteristic ready for writing...\n");           
            }
            show_characteristic_once = false;}
            break;
        case GATT_EVENT_QUERY_COMPLETE:

               if (service_found) gatt_client_discover_characteristics_for_service(handle_gatt_client_event, connection_handle, &my_service);        
            break;
        default:
            break;
    }}

static void handle_hci_event(uint8_t packet_type, uint16_t channel, uint8_t *packet, uint16_t size){
    UNUSED(channel);
    UNUSED(size);

    if (packet_type != HCI_EVENT_PACKET) return;
    advertising_report_t report;
    
    uint8_t event = hci_event_packet_get_type(packet);
    switch (event) {
        case BTSTACK_EVENT_STATE:
            // BTstack activated, get started
            if (btstack_event_state_get_state(packet) != HCI_STATE_WORKING) break;
            printf("BTstack activated, start scanning!\n");
            gap_set_scan_parameters(0,0x0030, 0x0030);
            gap_start_scan();
            break;
        case GAP_EVENT_ADVERTISING_REPORT:
            fill_advertising_report_from_packet(&report, packet);
            //dump_advertising_report(&report);
            if (found_device(&report)){
              //printf("********************found device\n");
              gap_stop_scan();
              gap_connect(report.address,report.address_type);}
            break;
        case HCI_EVENT_META_GAP:
            if (hci_event_gap_meta_get_subevent_code(packet) !=  GAP_SUBEVENT_LE_CONNECTION_COMPLETE) break;
            //printf("\nGATT browser - CONNECTED\n");
            connection_handle = gap_subevent_le_connection_complete_get_connection_handle(packet);
            gatt_client_discover_primary_services(handle_gatt_client_event, connection_handle);
            break;
        case HCI_EVENT_DISCONNECTION_COMPLETE:
            //printf("\nGATT browser - DISCONNECTED\n");
            break;
        default:
            break;
    }}

///////////////////////////////////////
static void gatt_client_setup(void){
  l2cap_init();
  gatt_client_init();
  sm_init();
  sm_set_io_capabilities(IO_CAPABILITY_NO_INPUT_NO_OUTPUT);
  hci_event_callback_registration.callback = &handle_hci_event;
  hci_add_event_handler(&hci_event_callback_registration);
}

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

static PT_THREAD (ui_thread(struct pt *pt)){
  PT_BEGIN(pt);
  static char message;
  while(1){
    sprintf(pt_serial_out_buffer, "type ON, OFF, ON13, OFF13\n\r");
    serial_write;
    serial_read;
    
     int status = gatt_client_write_value_of_characteristic_without_response(connection_handle, characteristic.value_handle, strlen(pt_serial_in_buffer), pt_serial_in_buffer);
     PT_YIELD_usec(500000);
     }
   PT_END(pt);}
     
 
int main() {
  stdio_init_all();
  printf("starting gatt client...\n");
  if (cyw43_arch_init()) {return -1; }
  gatt_client_setup();
  att_server_init(NULL, NULL, NULL);
  hci_power_control(HCI_POWER_ON);

  
    pt_add_thread(blink_thread) ;
    pt_add_thread(ui_thread);
    pt_sched_method = SCHED_ROUND_ROBIN ;
    pt_schedule_start ;

}
