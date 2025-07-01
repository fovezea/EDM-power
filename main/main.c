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

// Speed and leadscrew pitch settings
double jog_speed_mm_per_s = 1; // Speed in mm/s, can be set from elsewhere
double cut_speed_mm_per_s = 0.1; // Speed in mm/s, can be set from elsewhere
double leadscrew_pitch_mm = 4.0; // Leadscrew pitch in mm/rev
double steps_per_rev = 200; // Pulses per revolution (e.g., 200 for 1.8 degree stepper)

static const char *TAG = "main";

#include "freertos/queue.h"
QueueHandle_t pwm_adc_queue = NULL;

// Only extern variables that are defined in other files and actually used in main.c
extern volatile uint32_t last_capture_ticks;
extern volatile int adc_value_on_capture;
extern SemaphoreHandle_t capture_semaphore;

// Extern declaration for adc_on_capture_task (defined in ADC.c)
extern void adc_on_capture_task(void *pvParameters);
extern void mcpwm_halfbridge_task(void *pvParameters);
extern void adc_oneshot_init(void); // Add extern for ADC init

// Local static/global variables (defined in this file and actually used)
static rmt_channel_handle_t motor_chan;
static rmt_encoder_handle_t accel_motor_encoder;
static rmt_encoder_handle_t uniform_motor_encoder;
static rmt_encoder_handle_t decel_motor_encoder;

// The task function
void stepper_task(void *pvParameters)
{
 
    ////////////////////////
    // Configure EN + DIR as outputs
    ESP_LOGI(TAG, "Initialize EN + DIR GPIO");
    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR | 1ULL << STEP_MOTOR_GPIO_EN,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    // Configure jog and limit GPIOs
    ESP_LOGI(TAG, "Initialize jog and limit GPIOs");
    gpio_config_t jog_gpio_config = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << JOG_UP_GPIO) | (1ULL << JOG_DOWN_GPIO),
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&jog_gpio_config));

    gpio_config_t limit_gpio_config = {
        .mode = GPIO_MODE_INPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = (1ULL << LIMIT_SWITCH_GPIO) | (1ULL << START_CUT_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&limit_gpio_config));

    //calculate stepper frequency from mm/s
    double jog_freq_hz = stepper_calc_freq_from_speed(jog_speed_mm_per_s, steps_per_rev, leadscrew_pitch_mm);
    ESP_LOGI(TAG, "Calculated stepper jog frequency: %.2f Hz", jog_freq_hz);
    //calculate cut frequency from mm/s
    double cut_freq_hz = stepper_calc_freq_from_speed(cut_speed_mm_per_s, steps_per_rev, leadscrew_pitch_mm);
    ESP_LOGI(TAG, "Calculated stepper cut frequency: %.2f Hz", cut_freq_hz);


    // Create RMT TX channel
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
    stepper_motor_curve_encoder_config_t accel_encoder_config = {0};
    accel_encoder_config.resolution = STEP_MOTOR_RESOLUTION_HZ;
    accel_encoder_config.sample_points = 500;
    accel_encoder_config.start_freq_hz = 500;
    accel_encoder_config.end_freq_hz = 1500;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&accel_encoder_config, &accel_motor_encoder));
    if (accel_motor_encoder == NULL) {
        ESP_LOGE(TAG, "Failed to create accel_motor_encoder");
        return;
    }

    // Jog encoder: use curve encoder for configurable jog speed
    static rmt_encoder_handle_t jog_motor_encoder = NULL;
    stepper_motor_curve_encoder_config_t jog_curve_config = {0};
    jog_curve_config.resolution = STEP_MOTOR_RESOLUTION_HZ;
    jog_curve_config.sample_points = 2; // For constant speed, must be <= |start_freq_hz - end_freq_hz|
    jog_curve_config.start_freq_hz = 3000; // Set jog speed here (Hz)
    jog_curve_config.end_freq_hz = 2998;   // Must not be equal to start_freq_hz, and difference >= sample_points
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&jog_curve_config, &jog_motor_encoder));
    if (jog_motor_encoder == NULL) {
        ESP_LOGE(TAG, "Failed to create jog_motor_encoder");
        return;
    }

    stepper_motor_uniform_encoder_config_t uniform_encoder_config = {0};
    uniform_encoder_config.resolution = STEP_MOTOR_RESOLUTION_HZ;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_uniform_encoder(&uniform_encoder_config, &uniform_motor_encoder));
    if (uniform_motor_encoder == NULL) {
        ESP_LOGE(TAG, "Failed to create uniform_motor_encoder");
        return;
    }

    stepper_motor_curve_encoder_config_t decel_encoder_config = {0};
    decel_encoder_config.resolution = STEP_MOTOR_RESOLUTION_HZ;
    decel_encoder_config.sample_points = 500;
    decel_encoder_config.start_freq_hz = 1500;
    decel_encoder_config.end_freq_hz = 500;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_motor_encoder));
    if (decel_motor_encoder == NULL) {
        ESP_LOGE(TAG, "Failed to create decel_motor_encoder");
        return;
    }

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
    const int LOW_VOLTAGE = 500;            // adjust based on your ADC scaling
    const int HIGH_VOLTAGE = 2000;          // adjust based on your ADC scaling

    // ESP_LOGI(TAG, "RMT channel enabled, entering main loop");

    rmt_transmit_config_t tx_config = { .loop_count = 0 };
    int jogging = 0; // 0: not jogging, 1: up, -1: down
    bool encoder_running = false; // Track if encoder is running
    while (1) {
        int jog_up = gpio_get_level(JOG_UP_GPIO);
        int jog_down = gpio_get_level(JOG_DOWN_GPIO);
        int limit_switch = gpio_get_level(LIMIT_SWITCH_GPIO); // 1 = OK, 0 = limit hit
        int start_cut = gpio_get_level(START_CUT_GPIO);       // 1 = start, 0 = stop

      
        // If limit switch is OFF, inhibit all movement
        if (!limit_switch) {
            ESP_LOGI(TAG, "Limit switch hit, stopping all movement");
            // Stop the encoder if running
            if (encoder_running) {
                rmt_disable(motor_chan);
                encoder_running = false;
            }
            vTaskDelay(pdMS_TO_TICKS(20)); // Yield to avoid WDT and CPU hogging
            continue;
        }


             // if limit switch is OFF, inhibit Jog UP for now
        if (jog_up ) {
            ESP_LOGI(TAG, "Jog UP pressed");
            jogging = 1;
            gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE); // Retract direction
            uint32_t steps = 10; // More steps for faster jog
            ESP_LOGI(TAG, "Jog UP: accel phase");
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &steps, sizeof(steps), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            encoder_running = true;
            // Keep jogging at constant speed while button is held
            while (gpio_get_level(JOG_UP_GPIO) && gpio_get_level(LIMIT_SWITCH_GPIO)) {
                ESP_LOGI(TAG, "Jog UP: uniform phase");

                //ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
               ESP_ERROR_CHECK(rmt_transmit(motor_chan, jog_motor_encoder, &steps, sizeof(steps), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            ESP_LOGI(TAG, "Jog UP: decel phase");
            steps = 10;
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &steps, sizeof(steps), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            jogging = 0;
        } else if (jog_down) {
            ESP_LOGI(TAG, "Jog DOWN pressed");
            jogging = -1;
            gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
            uint32_t steps = 10; // More steps for faster jog
            ESP_LOGI(TAG, "Jog DOWN: accel phase");
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &steps, sizeof(steps), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            encoder_running = true;
            // Keep jogging at constant speed while button is held
            while (gpio_get_level(JOG_DOWN_GPIO) && gpio_get_level(LIMIT_SWITCH_GPIO)) {
                ESP_LOGI(TAG, "Jog DOWN: uniform phase");
                //ESP_ERROR_CHECK(rmt_transmit(motor_chan, uniform_motor_encoder, &uniform_speed_hz, sizeof(uniform_speed_hz), &tx_config));
                ESP_ERROR_CHECK(rmt_transmit(motor_chan, jog_motor_encoder, &steps, sizeof(steps), &tx_config));
                //ESP_ERROR_CHECK(rmt_transmit(motor_chan, jog_motor_encoder, &steps, sizeof(steps), &tx_config));
                ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
                vTaskDelay(pdMS_TO_TICKS(1));
            }
            ESP_LOGI(TAG, "Jog DOWN: decel phase");
            steps = 10;
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &steps, sizeof(steps), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            encoder_running = true;
            jogging = 0;
        } else if (!jogging && limit_switch && start_cut) {
            ESP_LOGI(TAG, "Start EDM cut");
            // int delay_ticks = 0; // Removed: delay_ticks no longer used
            int gap_voltage = adc_value_on_capture;
            ESP_LOGI(TAG, "DEBUG: gap_voltage=%d", gap_voltage); // Debug print
            // Control logic
            int step_direction = 0;
            if (gap_voltage < LOW_VOLTAGE) {
                step_direction = -1;
                 ESP_LOGI(TAG, "EDM: Too close, retracting electrode");
            } else if (gap_voltage > HIGH_VOLTAGE) {
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
                if (motor_chan == NULL || uniform_motor_encoder == NULL) {
                    ESP_LOGE(TAG, "motor_chan or uniform_motor_encoder is NULL!");
                } else {
                    esp_err_t tx_err = rmt_transmit(motor_chan, uniform_motor_encoder, &steps, sizeof(steps), &tx_config);
                    if (tx_err != ESP_OK) {
                        ESP_LOGE(TAG, "rmt_transmit failed: %s", esp_err_to_name(tx_err));
                    }
                    ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
                }
                encoder_running = true;
            } else if (step_direction == 1) {
                gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
                uint32_t steps = 5;
                if (motor_chan == NULL || uniform_motor_encoder == NULL) {
                    ESP_LOGE(TAG, "motor_chan or uniform_motor_encoder is NULL!");
                } else {
                    esp_err_t tx_err = rmt_transmit(motor_chan, uniform_motor_encoder, &steps, sizeof(steps), &tx_config);
                    if (tx_err != ESP_OK) {
                        ESP_LOGE(TAG, "rmt_transmit failed: %s", esp_err_to_name(tx_err));
                    }
                    ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
                }
                encoder_running = true;
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

    adc_oneshot_init(); // Initialize ADC before starting ADC task
    xTaskCreate(adc_on_capture_task, "adc_on_capture_task", 2048, NULL, 10, NULL); // High priority for fast ADC

    // Start MCPWM task for power train
    xTaskCreate(mcpwm_halfbridge_task, "mcpwm_halfbridge_task", 4096, NULL, 5, NULL);
}
