#include "esp_log.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_adc/adc_oneshot.h"

static const char *TAG = "adc_cali";

extern SemaphoreHandle_t capture_semaphore;
adc_oneshot_unit_handle_t adc_handle = NULL;
volatile int adc_value_on_capture = 0;

#define FILTER_WINDOW 8 // Number of samples for moving average

// ADC filtering and capture task
void adc_on_capture_task(void *pvParameters)
{
    int adc_buffer[FILTER_WINDOW] = {0};
    int buffer_index = 0;
    int last_valid = 0;
    const int MAX_JUMP = 200; // Max allowed jump between samples
    while (1) {
        if (capture_semaphore && xSemaphoreTake(capture_semaphore, portMAX_DELAY) == pdTRUE) {
            int value = 0;
            esp_err_t err = adc_oneshot_read(adc_handle, ADC_CHANNEL_6, &value);
            if (err == ESP_OK) {
                // Simple outlier filter: if value is too far from last valid, ignore
                if (buffer_index > 0 && abs(value - last_valid) > MAX_JUMP) {
                    value = last_valid; // Clamp to last valid
                }
                adc_buffer[buffer_index] = value;
                buffer_index = (buffer_index + 1) % FILTER_WINDOW;
                // Compute moving average
                int sum = 0;
                for (int i = 0; i < FILTER_WINDOW; ++i) sum += adc_buffer[i];
                int avg = sum / FILTER_WINDOW;
                adc_value_on_capture = avg;
                last_valid = value;
                ESP_LOGI(TAG, "ADC value (filtered avg): %d", adc_value_on_capture);
            } else {
                ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(err));
            }
        }
    }
}

// Only keep calibration init and deinit functions, as used in main.c
bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle)
{
    adc_cali_handle_t handle = NULL;
    esp_err_t ret = ESP_FAIL;
    bool calibrated = false;

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Curve Fitting");
        adc_cali_curve_fitting_config_t cali_config = {
            .unit_id = unit,
            .chan = channel,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_curve_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

#if ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    if (!calibrated) {
        ESP_LOGI(TAG, "calibration scheme version is %s", "Line Fitting");
        adc_cali_line_fitting_config_t cali_config = {
            .unit_id = unit,
            .atten = atten,
            .bitwidth = ADC_BITWIDTH_DEFAULT,
        };
        ret = adc_cali_create_scheme_line_fitting(&cali_config, &handle);
        if (ret == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    *out_handle = handle;
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Calibration Success");
    } else if (ret == ESP_ERR_NOT_SUPPORTED || !calibrated) {
        ESP_LOGW(TAG, "eFuse not burnt, skip software calibration");
    } else {
        ESP_LOGE(TAG, "Invalid arg or no memory");
    }

    return calibrated;
}
//not used 
void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));
#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
}