/*
 * Web Server with WebSocket for EDM Power Project
 * ESP-IDF v5.4.1 compatible
 */

#pragma once

#include "esp_http_server.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "cJSON.h"

#ifdef __cplusplus
extern "C" {
#endif

// Web server configuration
#define WEB_SERVER_PORT 80
#define MAX_CLIENTS 4
#define WEBSOCKET_BUFFER_SIZE 1024

// EDM Parameters that can be monitored/controlled
typedef struct {
    float duty_percent;           // PWM duty cycle
    int adc_value;               // Gap voltage ADC reading
    float gap_voltage;           // Calculated gap voltage
    bool is_cutting;             // EDM cutting status
    bool limit_switch_triggered; // Limit switch status
    float jog_speed;             // Jog speed in mm/s
    float cut_speed;             // Cut speed in mm/s
    int step_position;           // Current stepper position
    bool wifi_connected;         // WiFi connection status
    uint32_t uptime_seconds;     // System uptime
    int adc_blanking_delay;      // ADC lead edge blanking delay in ticks
    // PID parameters
    float pid_kp;                // Proportional gain
    float pid_ki;                // Integral gain
    float pid_kd;                // Derivative gain
    int pid_setpoint;            // Target ADC value
} edm_status_t;

// WebSocket client structure
typedef struct {
    int fd;
    bool is_websocket;
    char buffer[WEBSOCKET_BUFFER_SIZE];
    size_t buffer_len;
} client_t;

// Function declarations
esp_err_t web_server_init(void);
void web_server_deinit(void);
esp_err_t websocket_send_status(edm_status_t *status);
esp_err_t websocket_send_message(const char *message);
void web_server_update_status(edm_status_t *status);
esp_err_t web_server_start(void);
esp_err_t web_server_stop(void);
void web_server_task(void *pvParameters);

// Control functions from main.c
bool get_web_control_active(void);
void set_web_control_active(bool active);
bool get_edm_cutting_status(void);

// External variables that will be updated by main application
extern volatile int duty_percent;
extern volatile int adc_value_on_capture;
extern volatile int adc_blanking_delay_ticks;
extern volatile float pid_kp;
extern volatile float pid_ki;
extern volatile float pid_kd;
extern volatile int pid_setpoint;
extern volatile bool pid_control_enabled;
extern SemaphoreHandle_t capture_semaphore;

#ifdef __cplusplus
}
#endif 