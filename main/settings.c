#include "settings.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "settings";
static nvs_handle_t settings_nvs_handle = 0;

// NVS namespace for EDM settings
#define NVS_NAMESPACE "edm_settings"

esp_err_t settings_init(void) {
    esp_err_t ret = nvs_open(NVS_NAMESPACE, NVS_READWRITE, &settings_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS handle: %s", esp_err_to_name(ret));
        return ret;
    }
    ESP_LOGI(TAG, "Settings initialized successfully");
    return ESP_OK;
}

esp_err_t settings_load(edm_settings_t *settings) {
    if (!settings || !settings_nvs_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret;
    size_t required_size = sizeof(edm_settings_t);
    
    // Try to load settings
    ret = nvs_get_blob(settings_nvs_handle, "settings", settings, &required_size);
    
    if (ret == ESP_ERR_NVS_NOT_FOUND) {
        // Settings not found, use defaults
        ESP_LOGW(TAG, "Settings not found, using defaults");
        settings->duty_percent = DEFAULT_DUTY_PERCENT;
        settings->adc_blanking_delay = DEFAULT_ADC_BLANKING_DELAY;
        settings->jog_speed = DEFAULT_JOG_SPEED;
        settings->cut_speed = DEFAULT_CUT_SPEED;
        settings->pid_kp = DEFAULT_PID_KP;
        settings->pid_ki = DEFAULT_PID_KI;
        settings->pid_kd = DEFAULT_PID_KD;
        settings->pid_setpoint = DEFAULT_PID_SETPOINT;
        settings->pid_control_enabled = DEFAULT_PID_CONTROL_ENABLED;
        
        // Save defaults
        settings_save(settings);
        return ESP_OK;
    } else if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error loading settings: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Settings loaded successfully");
    ESP_LOGI(TAG, "Duty: %d%%, ADC Delay: %d ticks, Jog: %.1f mm/s, Cut: %.2f mm/s",
             settings->duty_percent, settings->adc_blanking_delay, 
             settings->jog_speed, settings->cut_speed);
    ESP_LOGI(TAG, "PID: Kp=%.3f, Ki=%.3f, Kd=%.3f, Setpoint=%d, Enabled=%s",
             settings->pid_kp, settings->pid_ki, settings->pid_kd, settings->pid_setpoint,
             settings->pid_control_enabled ? "true" : "false");
    
    return ESP_OK;
}

esp_err_t settings_save(const edm_settings_t *settings) {
    if (!settings || !settings_nvs_handle) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t ret = nvs_set_blob(settings_nvs_handle, "settings", settings, sizeof(edm_settings_t));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error saving settings: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ret = nvs_commit(settings_nvs_handle);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Error committing settings: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "Settings saved successfully");
    ESP_LOGI(TAG, "Duty: %d%%, ADC Delay: %d ticks, Jog: %.1f mm/s, Cut: %.2f mm/s",
             settings->duty_percent, settings->adc_blanking_delay, 
             settings->jog_speed, settings->cut_speed);
    ESP_LOGI(TAG, "PID: Kp=%.3f, Ki=%.3f, Kd=%.3f, Setpoint=%d, Enabled=%s",
             settings->pid_kp, settings->pid_ki, settings->pid_kd, settings->pid_setpoint,
             settings->pid_control_enabled ? "true" : "false");
    
    return ESP_OK;
}

esp_err_t settings_reset_to_defaults(void) {
    edm_settings_t default_settings = {
        .duty_percent = DEFAULT_DUTY_PERCENT,
        .adc_blanking_delay = DEFAULT_ADC_BLANKING_DELAY,
        .jog_speed = DEFAULT_JOG_SPEED,
        .cut_speed = DEFAULT_CUT_SPEED,
        .pid_kp = DEFAULT_PID_KP,
        .pid_ki = DEFAULT_PID_KI,
        .pid_kd = DEFAULT_PID_KD,
        .pid_setpoint = DEFAULT_PID_SETPOINT,
        .pid_control_enabled = DEFAULT_PID_CONTROL_ENABLED
    };
    
    esp_err_t ret = settings_save(&default_settings);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Settings reset to defaults");
    }
    return ret;
}

void settings_deinit(void) {
    if (settings_nvs_handle) {
        nvs_close(settings_nvs_handle);
        settings_nvs_handle = 0;
        ESP_LOGI(TAG, "Settings deinitialized");
    }
} 