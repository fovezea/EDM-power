/*
 * SPDX-FileCopyrightText: 2021-2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/rmt_tx.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "stepper_motor_encoder.h"
#include "freertos/semphr.h"

//#include "driver/adc.h"
//#include "esp_adc_cal.h"
//#include "/adc_cali.h"
//#include "/adc_cali_scheme.h"
//#include "esp_adc/adc_oneshot.h"
//#include "esp_adc_cal.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

///////////////////////////////Change the following configurations according to your board//////////////////////////////
#define STEP_MOTOR_GPIO_EN       0
#define STEP_MOTOR_GPIO_DIR      2
#define STEP_MOTOR_GPIO_STEP     4
#define STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE
#define HYSTERESIS_WIDTH 100 // ADC value dead zone width

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution
adc_cali_handle_t adc1_cali_chan6_handle = NULL; // Global handle for ADC calibration
bool do_calibration1_chan6 = false; // Global variable to control ADC calibration

static const char *TAG = "main";

#include "freertos/queue.h"
extern QueueHandle_t pwm_adc_queue;
extern bool do_calibration1_chan6; // Global variable to control ADC calibration

extern volatile uint32_t last_capture_ticks;
extern SemaphoreHandle_t capture_semaphore;

extern adc_cali_handle_t adc1_cali_chan6_handle; // Global handle for ADC
extern bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);
static rmt_channel_handle_t motor_chan;
static rmt_encoder_handle_t accel_motor_encoder;
static rmt_encoder_handle_t uniform_motor_encoder;
static rmt_encoder_handle_t decel_motor_encoder;
//static rmt_transmit_config_t tx_config;
//extern esp_adc_cal_characteristics_t adc_chars;
extern adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };

//const static uint32_t accel_samples = 500;
//const static uint32_t decel_samples = 500;

//const static uint32_t uniform_speed_hz = 1500;

//extern adc_oneshot_unit_handle_t adc_handle; 
//volatile int g_pwm_adc_value = 0;

extern void halfbridge_pwm_task(void *pvParameters); // Forward declaration of the PWM task
extern void mcpwm_halfbridge_task(void *pvParameters); // Forward declaration of the MCPWM task

// The task function
void stepper_task(void *pvParameters)
{
 
    ////////////////////////
ESP_LOGI(TAG, "Initialize EN + DIR GPIO");
    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR | 1ULL << STEP_MOTOR_GPIO_EN,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    ESP_LOGI(TAG, "Create RMT TX channel");
    //rmt_channel_handle_t motor_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

    ESP_LOGI(TAG, "Set spin direction");
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    ESP_LOGI(TAG, "Enable step motor");
    gpio_set_level(STEP_MOTOR_GPIO_EN, STEP_MOTOR_ENABLE_LEVEL);

    ESP_LOGI(TAG, "Create motor encoders");
    stepper_motor_curve_encoder_config_t accel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 500,
        .end_freq_hz = 1500,
    };
    //rmt_encoder_handle_t accel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_motor_encoder));

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
    };
    //rmt_encoder_handle_t uniform_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));

    stepper_motor_curve_encoder_config_t decel_encoder_config = {
        .resolution = STEP_MOTOR_RESOLUTION_HZ,
        .sample_points = 500,
        .start_freq_hz = 1500,
        .end_freq_hz = 500,
    };
    //rmt_encoder_handle_t decel_motor_encoder = NULL;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_motor_encoder));

    ESP_LOGI(TAG, "Enable RMT channel");
    // Debug: print motor_chan handle before enabling
    ESP_LOGI(TAG, "motor_chan handle: %p", motor_chan);
    esp_err_t err = rmt_enable(motor_chan);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable RMT channel: %s", esp_err_to_name(err));
        return;
    }
    // Variable declarations moved to function scope
    extern volatile uint32_t last_capture_ticks;
    extern volatile int adc_value_on_capture;
    const uint32_t SHORT_DELAY_TICKS = 100; // adjust based on your timing
    const int LOW_VOLTAGE = 500;            // adjust based on your ADC scaling
    const uint32_t LONG_DELAY_TICKS = 500;  // adjust based on your timing
    const int HIGH_VOLTAGE = 2000;          // adjust based on your ADC scaling
    int step_direction = 0; // -1: retract, 0: hold, 1: advance

    ESP_LOGI(TAG, "RMT channel enabled, entering main loop");
    while (1) {
        // Non-blocking: check for new capture event and ADC sample
        if (capture_semaphore && xSemaphoreTake(capture_semaphore, 0) == pdTRUE) {
            uint32_t delay_ticks = last_capture_ticks;
            int gap_voltage = adc_value_on_capture;
            ESP_LOGI(TAG, "EDM: delay_ticks=%lu, gap_voltage=%d", delay_ticks, gap_voltage);

            // Control logic
            if (delay_ticks < SHORT_DELAY_TICKS && gap_voltage < LOW_VOLTAGE) {
                step_direction = -1;
                ESP_LOGI(TAG, "EDM: Too close, retracting electrode");
            } else if (delay_ticks > LONG_DELAY_TICKS && gap_voltage > HIGH_VOLTAGE) {
                step_direction = 1;
                ESP_LOGI(TAG, "EDM: Too far, advancing electrode");
            } else {
                step_direction = 0;
                ESP_LOGI(TAG, "EDM: Gap OK, holding position");
            }
            /*
            // Move stepper based on step_direction
            if (step_direction == -1) {
                gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
                uint32_t steps = 5;
                ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &steps, sizeof(steps), NULL));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            } else if (step_direction == 1) {
                gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
                uint32_t steps = 5;
                ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &steps, sizeof(steps), NULL));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            } // else hold (do nothing)
            */
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // Always yield to avoid WDT
    }

}

extern adc_oneshot_unit_handle_t adc_handle;
volatile int adc_value_on_capture = 0;

#define FILTER_WINDOW 8 // Number of samples for moving average
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

void app_main(void)
{
    pwm_adc_queue = xQueueCreate(1, sizeof(int));
    // Create the task
    xTaskCreate(stepper_task, "stepper_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Stepper motor example started");
    // create pwm task
    // xTaskCreate(halfbridge_pwm_task, "halfbridge_pwm_task", 4096, NULL, 5, NULL);
    xTaskCreate(mcpwm_halfbridge_task, "mcpwm_halfbridge", 4096, NULL, 5, NULL);
    xTaskCreate(adc_on_capture_task, "adc_on_capture_task", 2048, NULL, 10, NULL); // High priority for fast ADC
}
