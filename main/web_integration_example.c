/*
 * Web Server Integration Example for EDM Power Project
 * This file shows how to integrate the web server with your main application
 */

#include "web_server.h"
#include "wifi_connect.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

static const char *TAG = "web_integration";

// Global variables for EDM status
extern volatile int duty_percent;
extern volatile int adc_value_on_capture;
extern SemaphoreHandle_t capture_semaphore;

// EDM control flags
static volatile bool edm_cutting_enabled = false;
static volatile bool jog_up_requested = false;
static volatile bool jog_down_requested = false;
static volatile bool home_position_requested = false;

// Queue for commands from web interface
static QueueHandle_t command_queue = NULL;

// Command structure
typedef struct {
    char command[32];
    int value;
} web_command_t;

// Web server integration task
void web_integration_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Web integration task started");
    
    // Create command queue
    command_queue = xQueueCreate(10, sizeof(web_command_t));
    
    // Initialize web server
    esp_err_t ret = web_server_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start web server");
        return;
    }
    
    // Create web server task
    xTaskCreate(web_server_task, "web_server_task", 8192, NULL, 4, NULL);
    
    ESP_LOGI(TAG, "Web server started successfully");
    ESP_LOGI(TAG, "Access the EDM control interface at: http://[ESP32_IP_ADDRESS]");
    
    // Main integration loop
    while (1) {
        // Process commands from web interface
        web_command_t cmd;
        if (xQueueReceive(command_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
            ESP_LOGI(TAG, "Processing web command: %s, value: %d", cmd.command, cmd.value);
            
            if (strcmp(cmd.command, "set_duty") == 0) {
                duty_percent = cmd.value;
                ESP_LOGI(TAG, "Duty cycle set to %d%% via web interface", cmd.value);
            } else if (strcmp(cmd.command, "start_cut") == 0) {
                edm_cutting_enabled = true;
                ESP_LOGI(TAG, "EDM cutting started via web interface");
            } else if (strcmp(cmd.command, "stop_cut") == 0) {
                edm_cutting_enabled = false;
                ESP_LOGI(TAG, "EDM cutting stopped via web interface");
            } else if (strcmp(cmd.command, "jog_up") == 0) {
                jog_up_requested = true;
                ESP_LOGI(TAG, "Jog up requested via web interface");
            } else if (strcmp(cmd.command, "jog_down") == 0) {
                jog_down_requested = true;
                ESP_LOGI(TAG, "Jog down requested via web interface");
            } else if (strcmp(cmd.command, "home_position") == 0) {
                home_position_requested = true;
                ESP_LOGI(TAG, "Home position requested via web interface");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }
}

// Function to send commands to the integration task
esp_err_t send_web_command(const char *command, int value) {
    if (command_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    
    web_command_t cmd;
    strncpy(cmd.command, command, sizeof(cmd.command) - 1);
    cmd.command[sizeof(cmd.command) - 1] = '\0';
    cmd.value = value;
    
    if (xQueueSend(command_queue, &cmd, pdMS_TO_TICKS(100)) == pdTRUE) {
        return ESP_OK;
    } else {
        return ESP_ERR_TIMEOUT;
    }
}

// Getter functions for EDM status
bool get_edm_cutting_status(void) {
    return edm_cutting_enabled;
}

bool get_jog_up_requested(void) {
    bool requested = jog_up_requested;
    jog_up_requested = false; // Clear the request
    return requested;
}

bool get_jog_down_requested(void) {
    bool requested = jog_down_requested;
    jog_down_requested = false; // Clear the request
    return requested;
}

bool get_home_position_requested(void) {
    bool requested = home_position_requested;
    home_position_requested = false; // Clear the request
    return requested;
}

// Example of how to modify your main.c to integrate with the web server:
/*
void app_main(void)
{
    // Your existing initialization code
    pwm_adc_queue = xQueueCreate(1, sizeof(int));
    xTaskCreate(stepper_task, "stepper_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Stepper motor example started");

    adc_oneshot_init();
    xTaskCreate(adc_on_capture_task, "adc_on_capture_task", 2048, NULL, 10, NULL);
    xTaskCreate(mcpwm_halfbridge_task, "mcpwm_halfbridge_task", 4096, NULL, 5, NULL);
    
    // Initialize WiFi first
    xTaskCreate(wifi_example_task, "wifi_example_task", 4096, NULL, 3, NULL);
    ESP_LOGI(TAG, "WiFi task started");
    
    // Start web server integration
    xTaskCreate(web_integration_task, "web_integration_task", 8192, NULL, 2, NULL);
    ESP_LOGI(TAG, "Web integration task started");
}

// In your stepper_task, you can now check for web commands:
void stepper_task(void *pvParameters)
{
    // ... existing initialization code ...
    
    while (1) {
        // Check for web interface commands
        if (get_jog_up_requested()) {
            // Execute jog up command
            ESP_LOGI(TAG, "Executing jog up from web interface");
            // Your jog up logic here
        }
        
        if (get_jog_down_requested()) {
            // Execute jog down command
            ESP_LOGI(TAG, "Executing jog down from web interface");
            // Your jog down logic here
        }
        
        if (get_home_position_requested()) {
            // Execute home position command
            ESP_LOGI(TAG, "Executing home position from web interface");
            // Your home position logic here
        }
        
        // Check if EDM cutting is enabled via web interface
        if (get_edm_cutting_status()) {
            // Your EDM cutting logic here
            // This replaces the manual start_cut button logic
        }
        
        // ... rest of your existing stepper task code ...
    }
}
*/ 