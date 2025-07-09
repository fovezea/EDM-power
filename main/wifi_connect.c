/*
 * WiFi Connection Module for EDM Power Project
 * ESP-IDF v5.4.1 compatible
 */

#include "wifi_connect.h"
#include "esp_netif.h"
#include "esp_system.h"
#include "nvs_flash.h"

static const char *TAG = "wifi_connect";

// Event group to signal when we are connected
static EventGroupHandle_t s_wifi_event_group;

// The event group allows multiple bits for each event, but we only care about two events:
// - we are connected to the AP with an IP
// - we failed to connect after the maximum amount of re-tries
static int s_retry_num = 0;
static bool s_wifi_connected = false;

esp_err_t wifi_init_sta(void)
{
    esp_err_t ret = ESP_OK;
    
    // Initialize NVS
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");

    // Create the event group
    s_wifi_event_group = xEventGroupCreate();

    // Initialize TCP/IP stack
    ESP_ERROR_CHECK(esp_netif_init());

    // Create default event loop
    ESP_ERROR_CHECK(esp_event_loop_create_default());

    // Create default netif instance
    esp_netif_create_default_wifi_sta();

    // Initialize WiFi
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Register event handlers
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &ip_event_handler,
                                                        NULL,
                                                        NULL));

    // Configure WiFi
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASS,
            .threshold.authmode = WIFI_AUTH_WPA2_PSK,
            .pmf_cfg = {
                .capable = true,
                .required = false
            },
        },
    };

    // Set WiFi mode and start
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "wifi_init_sta finished.");

    // Wait until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
    // number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler()
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);

    // xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually happened.
    if (bits & WIFI_CONNECTED_BIT) {
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 WIFI_SSID, WIFI_PASS);
        s_wifi_connected = true;
        ret = ESP_OK;
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 WIFI_SSID, WIFI_PASS);
        s_wifi_connected = false;
        ret = ESP_FAIL;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        s_wifi_connected = false;
        ret = ESP_ERR_INVALID_STATE;
    }

    return ret;
}

void wifi_event_handler(void* arg, esp_event_base_t event_base,
                       int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        s_wifi_connected = false;
        if (s_retry_num < WIFI_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    }
}

void ip_event_handler(void* arg, esp_event_base_t event_base,
                     int32_t event_id, void* event_data)
{
    if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

bool wifi_is_connected(void)
{
    return s_wifi_connected;
}

esp_err_t wifi_disconnect(void)
{
    esp_err_t ret = esp_wifi_disconnect();
    if (ret == ESP_OK) {
        s_wifi_connected = false;
        ESP_LOGI(TAG, "WiFi disconnected");
    }
    return ret;
}

esp_err_t wifi_reconnect(void)
{
    s_retry_num = 0;
    s_wifi_connected = false;
    xEventGroupClearBits(s_wifi_event_group, WIFI_CONNECTED_BIT | WIFI_FAIL_BIT);
    
    esp_err_t ret = esp_wifi_connect();
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "WiFi reconnection initiated");
    }
    return ret;
} 