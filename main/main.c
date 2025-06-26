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
//buttons for jogging
#define JOG_UP_GPIO   12  // Choose your GPIO numbers
#define JOG_DOWN_GPIO 13
#define LIMIT_SWITCH_GPIO 14
#define START_CUT_GPIO    15

#define STEP_MOTOR_RESOLUTION_HZ 1000000 // 1MHz resolution

static const char *TAG = "main";

#include "freertos/queue.h"
QueueHandle_t pwm_adc_queue = NULL;

// Only extern variables that are defined in other files and actually used in main.c
extern volatile uint32_t last_capture_ticks;
extern volatile int adc_value_on_capture;
// capture_semaphore and adc_calibration_init are not used in main.c, so removed

// Extern declaration for adc_on_capture_task (defined in ADC.c)
extern void adc_on_capture_task(void *pvParameters);

// Local static/global variables (defined in this file and actually used)
static rmt_channel_handle_t motor_chan;
static rmt_encoder_handle_t accel_motor_encoder;
static rmt_encoder_handle_t uniform_motor_encoder;
static rmt_encoder_handle_t decel_motor_encoder;

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


    //ESP_LOGI(TAG, "Initialize  GPIO");
        gpio_config_t jog_gpio_config = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << JOG_UP_GPIO) | (1ULL << JOG_DOWN_GPIO) | (1ULL << LIMIT_SWITCH_GPIO) | (1ULL << START_CUT_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&jog_gpio_config));

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

    ESP_LOGI(TAG, "RMT channel enabled, entering main loop");

    int jogging = 0; // 0: not jogging, 1: up, -1: down
    while (1) {
        int jog_up = gpio_get_level(JOG_UP_GPIO);
        int jog_down = gpio_get_level(JOG_DOWN_GPIO);
        int limit_switch = gpio_get_level(LIMIT_SWITCH_GPIO); // 1 = OK, 0 = limit hit
        int start_cut = gpio_get_level(START_CUT_GPIO);       // 1 = start, 0 = stop

        // If limit switch is OFF, inhibit all movement
        if (!limit_switch) {
            // Optionally, stop the motor if running
            continue;
        }

        if (jog_up) {
            jogging = 1;
            gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
            uint32_t steps = 1;
         //   ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &steps, sizeof(steps), NULL));
         //   ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            // Keep jogging at constant speed while button is held
            while (gpio_get_level(JOG_UP_GPIO) && gpio_get_level(LIMIT_SWITCH_GPIO)) {
          //      ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &steps, sizeof(steps), NULL));
          //      ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
                vTaskDelay(pdMS_TO_TICKS(1));
            }
         //   ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &steps, sizeof(steps), NULL));
         //   ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            jogging = 0;
        } else if (jog_down) {
            jogging = -1;
          //  gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE);
            uint32_t steps = 1;
         //   ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &steps, sizeof(steps), NULL));
          //  ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            // Keep jogging at constant speed while button is held
            while (gpio_get_level(JOG_DOWN_GPIO) && gpio_get_level(LIMIT_SWITCH_GPIO)) {
             //   ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &steps, sizeof(steps), NULL));
             //   ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &steps, sizeof(steps), NULL));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            jogging = 0;
        } else if (!jogging && limit_switch && start_cut) {
            uint32_t delay_ticks = last_capture_ticks;
            int gap_voltage = adc_value_on_capture;
            ESP_LOGI(TAG, "EDM: delay_ticks=%lu, gap_voltage=%d", delay_ticks, gap_voltage);

            // Control logic
            int step_direction = 0;
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
            vTaskDelay(pdMS_TO_TICKS(20)); // Always yield to avoid WDT
        }
    }

}



void app_main(void)
{
    pwm_adc_queue = xQueueCreate(1, sizeof(int));
    // Create the task
    xTaskCreate(stepper_task, "stepper_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Stepper motor example started");

    xTaskCreate(adc_on_capture_task, "adc_on_capture_task", 2048, NULL, 10, NULL); // High priority for fast ADC
}
