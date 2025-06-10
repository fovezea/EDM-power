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

#include "driver/adc.h"
#include "esp_adc_cal.h"
//#include "/adc_cali.h"
//#include "/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
//#include "esp_adc_cal.h"

///////////////////////////////Change the following configurations according to your board//////////////////////////////
#define STEP_MOTOR_GPIO_EN       0
#define STEP_MOTOR_GPIO_DIR      2
#define STEP_MOTOR_GPIO_STEP     4
#define STEP_MOTOR_ENABLE_LEVEL  0 // DRV8825 is enabled on low level
#define STEP_MOTOR_SPIN_DIR_CLOCKWISE 0
#define STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE !STEP_MOTOR_SPIN_DIR_CLOCKWISE
#define HYSTERESIS_WIDTH 100 // ADC value dead zone width

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

static const char *TAG = "main";

#include "freertos/queue.h"
extern QueueHandle_t pwm_adc_queue;

static rmt_channel_handle_t motor_chan;
static rmt_encoder_handle_t accel_motor_encoder;
static rmt_encoder_handle_t uniform_motor_encoder;
static rmt_encoder_handle_t decel_motor_encoder;
//static rmt_transmit_config_t tx_config;
extern esp_adc_cal_characteristics_t adc_chars;
const static uint32_t accel_samples = 500;
const static uint32_t decel_samples = 500;

const static uint32_t uniform_speed_hz = 1500;

extern adc_oneshot_unit_handle_t adc_handle; 
//volatile int g_pwm_adc_value = 0;

extern void halfbridge_pwm_task(void *pvParameters); // Forward declaration of the PWM task

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
    ESP_ERROR_CHECK(rmt_enable(motor_chan));

    ESP_LOGI(TAG, "Spin motor for 6000 steps: 500 accel + 5000 uniform + 500 decel");
    rmt_transmit_config_t tx_config = {
        .loop_count = 0,
    };



    /////////////////////////
/*tepper_motor_curve_encoder_config_t accel_cfg = {
    .resolution = STEP_MOTOR_RESOLUTION_HZ,
    .sample_points = 500,
    .start_freq_hz = 500,
    .end_freq_hz = 1500,
};
stepper_motor_curve_encoder_config_t decel_cfg = {
    .resolution = STEP_MOTOR_RESOLUTION_HZ,
    .sample_points = 500,
    .start_freq_hz = 1500,
    .end_freq_hz = 500,
};
*/ 

    int pwm_adc_value = 0; // Local variable to hold ADC value retrieved from the queue
    bool rmt_enabled = true; // Assume enabled at startup (since enabled in app_main)

     while (1) {
        xQueueReceive(pwm_adc_queue, &pwm_adc_value, pdMS_TO_TICKS(10));
        uint32_t voltage = esp_adc_cal_raw_to_voltage(pwm_adc_value, &adc_chars);
        ESP_LOGI(TAG, "ADC Reading: %d, Voltage: %lu mV", pwm_adc_value, voltage);

        // Hysteresis dead zone
        int center = 2048;
        int lower = center - (HYSTERESIS_WIDTH / 2);
        int upper = center + (HYSTERESIS_WIDTH / 2);

        if (pwm_adc_value > lower && pwm_adc_value < upper) {
            // In dead zone: disable motor
            gpio_set_level(STEP_MOTOR_GPIO_EN, !STEP_MOTOR_ENABLE_LEVEL); // disable
            ESP_LOGI(TAG, "In dead zone: Motor stopped");
            // Disable RMT channel to stop sending pulses, only if enabled
            if (rmt_enabled) {
                ESP_ERROR_CHECK(rmt_disable(motor_chan));
                rmt_enabled = false;
                ESP_LOGI(TAG, "In dead zone: Motor stopped (RMT disabled)");
            }
        } else {
            // Outside dead zone: enable motor and set speed, only if disabled
            if (!rmt_enabled) {
                ESP_ERROR_CHECK(rmt_enable(motor_chan));
                rmt_enabled = true;
                ESP_LOGI(TAG, "Outside dead zone: Motor enabled (RMT enabled)");
            }
            // ...existing code for speed/direction/transmit...
            uint32_t speed_hz = (pwm_adc_value * 1500) / 4095;
            if (speed_hz < 100) speed_hz = 100;

            if (pwm_adc_value < lower) {
                gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
                ESP_LOGI(TAG, "Direction: Counter-clockwise");
            } else {
                gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
                ESP_LOGI(TAG, "Direction: Clockwise");
            }
            ESP_LOGI(TAG, "Mapped Speed: %lu Hz", speed_hz);

            gpio_set_level(STEP_MOTOR_GPIO_EN, STEP_MOTOR_ENABLE_LEVEL);

            ESP_LOGI(TAG, "Transmitting motor commands in three phases: acceleration, uniform speed, and deceleration");

           // tx_config.loop_count = 0;
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &accel_samples, sizeof(accel_samples), &tx_config));
            //ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, NULL, 0, &tx_config));
           // ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &accel_cfg, sizeof(accel_cfg), &tx_config));
            //ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &accel_samples, sizeof(accel_samples), &tx_config));

            // Do not set loop_count to 5000; always keep it 0 for stepper encoders
            //tx_config.loop_count = 5000;  
           // tx_config.loop_count = 0;

            //ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &accel_encoder_config, sizeof(accel_encoder_config), &tx_config));
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
           // tx_config.loop_count = 0;
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &decel_samples, sizeof(decel_samples), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
        }

        vTaskDelay(pdMS_TO_TICKS(1000));
    }

}

void app_main(void)
{
    
   pwm_adc_queue = xQueueCreate(1, sizeof(int));
    // Create the task
    xTaskCreate(stepper_task, "stepper_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Stepper motor example started");
    // create pwm task
     xTaskCreate(halfbridge_pwm_task, "halfbridge_pwm_task", 2048, NULL, 6, NULL);
}
