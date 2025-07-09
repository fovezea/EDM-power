/*
 * WiFi Connection Module for EDM Power Project
 * ESP-IDF v5.4.1 compatible
 */

#pragma once

#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"

#ifdef __cplusplus
extern "C" {
#endif

// WiFi configuration
//#define WIFI_SSID "403"  // Replace with your WiFi SSID
//#define WIFI_PASS "12345678"  // Replace with your WiFi password  
#define WIFI_SSID "apacalda"  // Replace with your WiFi SSID
#define WIFI_PASS "apacaldahaideshimdighel"  // Replace with your WiFi password 
#define WIFI_MAXIMUM_RETRY 5

// Event group to signal when we are connected
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

// Function declarations
esp_err_t wifi_init_sta(void);
void wifi_event_handler(void* arg, esp_event_base_t event_base,
                       int32_t event_id, void* event_data);
void ip_event_handler(void* arg, esp_event_base_t event_base,
                     int32_t event_id, void* event_data);
bool wifi_is_connected(void);
esp_err_t wifi_disconnect(void);
esp_err_t wifi_reconnect(void);

#ifdef __cplusplus
}
#endif 