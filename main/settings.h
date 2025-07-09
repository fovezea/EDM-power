/*
 * Settings Management for EDM Power Project
 * Uses NVS (Non-Volatile Storage) for persistent settings
 */

#pragma once

#include "esp_err.h"
#include "nvs_flash.h"
#include "nvs.h"

#ifdef __cplusplus
extern "C" {
#endif

// Settings structure
typedef struct {
    int duty_percent;           // PWM duty cycle (1-99)
    int adc_blanking_delay;     // ADC lead edge blanking delay (1-200 ticks)
    float jog_speed;            // Jog speed in mm/s
    float cut_speed;            // Cut speed in mm/s
    // PID control parameters
    float pid_kp;               // Proportional gain
    float pid_ki;               // Integral gain
    float pid_kd;               // Derivative gain
    int pid_setpoint;           // Target ADC value
    // Add more settings as needed
} edm_settings_t;

// Function declarations
esp_err_t settings_init(void);
esp_err_t settings_load(edm_settings_t *settings);
esp_err_t settings_save(const edm_settings_t *settings);
esp_err_t settings_reset_to_defaults(void);
void settings_deinit(void);

// Default settings
#define DEFAULT_DUTY_PERCENT 40
#define DEFAULT_ADC_BLANKING_DELAY 1
#define DEFAULT_JOG_SPEED 1.0f
#define DEFAULT_CUT_SPEED 0.1f
// PID default values
#define DEFAULT_PID_KP 0.05f
#define DEFAULT_PID_KI 0.01f
#define DEFAULT_PID_KD 0.0f
#define DEFAULT_PID_SETPOINT 1500

#ifdef __cplusplus
}
#endif 