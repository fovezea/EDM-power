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
#include "nvs_flash.h"

#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"

// Web server and WiFi includes
#include "wifi_connect.h"
#include "web_server.h"
#include "settings.h"
#include "freertos/queue.h"
#include <string.h>

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

// Web server integration variables
static volatile bool edm_cutting_enabled = false;
static volatile bool jog_up_requested = false;
static volatile bool jog_down_requested = false;
static volatile bool home_position_requested = false;
static volatile bool web_control_active = false; // Default: physical buttons in control
QueueHandle_t command_queue = NULL;

// Command structure for web interface
typedef struct {
    char command[32];
    int value;
} web_command_t;

// Web server task function declaration
void web_server_task(void *pvParameters);
extern esp_err_t web_server_init(void);

// Local static/global variables (defined in this file and actually used)
static rmt_channel_handle_t motor_chan;
static rmt_encoder_handle_t accel_motor_encoder;
static rmt_encoder_handle_t uniform_motor_encoder;
static rmt_encoder_handle_t decel_motor_encoder;

// Web server integration functions
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

bool get_web_control_active(void) {
    return web_control_active;
}

void set_web_control_active(bool active) {
    web_control_active = active;
    ESP_LOGI(TAG, "Control switched to: %s", active ? "WEB" : "PHYSICAL");
}

// Web server integration task
void web_integration_task(void *pvParameters)
{
    ESP_LOGI(TAG, "Web integration task started");
    
    // Create command queue
    command_queue = xQueueCreate(10, sizeof(web_command_t));
    
    // Initialize WiFi first
    esp_err_t wifi_ret = wifi_init_sta();
    if (wifi_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to connect to WiFi, web interface will not be available");
    } else {
        ESP_LOGI(TAG, "WiFi connected successfully");
        
        // Wait a bit for WiFi to stabilize
        vTaskDelay(pdMS_TO_TICKS(2000));
        
        // Initialize web server
        esp_err_t web_ret = web_server_start();
        if (web_ret != ESP_OK) {
            ESP_LOGE(TAG, "Failed to start web server, retrying...");
            // Wait and retry once
            vTaskDelay(pdMS_TO_TICKS(5000));
            web_ret = web_server_start();
            if (web_ret != ESP_OK) {
                ESP_LOGE(TAG, "Web server start failed after retry, continuing without web interface");
            } else {
                ESP_LOGI(TAG, "Web server started successfully after retry");
                xTaskCreate(web_server_task, "web_server_task", 8192, NULL, 4, NULL);
                ESP_LOGI(TAG, "Access the EDM control interface at: http://[ESP32_IP_ADDRESS]");
            }
        } else {
            ESP_LOGI(TAG, "Web server started successfully");
            xTaskCreate(web_server_task, "web_server_task", 8192, NULL, 4, NULL);
            ESP_LOGI(TAG, "Access the EDM control interface at: http://[ESP32_IP_ADDRESS]");
        }
    }
    
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
            } else if (strcmp(cmd.command, "web_control") == 0) {
                set_web_control_active(cmd.value != 0);
                ESP_LOGI(TAG, "Web control %s via web interface", cmd.value != 0 ? "enabled" : "disabled");
            }
        }
        
        vTaskDelay(pdMS_TO_TICKS(100)); // Check every 100ms
    }
}

// The task function
void stepper_task(void *pvParameters)
{
 
    ////////////////////////
    // Configure EN + DIR as outputs
    // ESP_LOGI(TAG, "Initialize EN + DIR GPIO");
    gpio_config_t en_dir_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .intr_type = GPIO_INTR_DISABLE,
        .pin_bit_mask = 1ULL << STEP_MOTOR_GPIO_DIR | 1ULL << STEP_MOTOR_GPIO_EN,
    };
    ESP_ERROR_CHECK(gpio_config(&en_dir_gpio_config));

    // Configure jog and limit GPIOs
    // ESP_LOGI(TAG, "Initialize jog and limit GPIOs");
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
    // ESP_LOGI(TAG, "Calculated stepper jog frequency: %.2f Hz", jog_freq_hz);
    //calculate cut frequency from mm/s
    double cut_freq_hz = stepper_calc_freq_from_speed(cut_speed_mm_per_s, steps_per_rev, leadscrew_pitch_mm);
    // ESP_LOGI(TAG, "Calculated stepper cut frequency: %.2f Hz", cut_freq_hz);


    // Create RMT TX channel
    // ESP_LOGI(TAG, "Create RMT TX channel");
    //rmt_channel_handle_t motor_chan = NULL;
    rmt_tx_channel_config_t tx_chan_config = {
        .clk_src = RMT_CLK_SRC_DEFAULT, // select clock source
        .gpio_num = STEP_MOTOR_GPIO_STEP,
        .mem_block_symbols = 64,
        .resolution_hz = STEP_MOTOR_RESOLUTION_HZ,
        .trans_queue_depth = 10, // set the number of transactions that can be pending in the background
    };
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_chan_config, &motor_chan));

    // ESP_LOGI(TAG, "Set spin direction");
    gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
    // ESP_LOGI(TAG, "Enable step motor");
    gpio_set_level(STEP_MOTOR_GPIO_EN, STEP_MOTOR_ENABLE_LEVEL);

    // ESP_LOGI(TAG, "Create motor encoders");
    stepper_motor_curve_encoder_config_t accel_encoder_config = {0};
    accel_encoder_config.resolution = STEP_MOTOR_RESOLUTION_HZ;
    accel_encoder_config.sample_points = 1000;
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
    jog_curve_config.start_freq_hz = 1500; // Increased jog speed (Hz)
    jog_curve_config.end_freq_hz = 1498;   // Must not be equal to start_freq_hz, and difference >= sample_points
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
    decel_encoder_config.sample_points = 1000;
    decel_encoder_config.start_freq_hz = 1500;
    decel_encoder_config.end_freq_hz = 500;
    ESP_ERROR_CHECK(rmt_new_stepper_motor_curve_encoder(&decel_encoder_config, &decel_motor_encoder));
    if (decel_motor_encoder == NULL) {
        ESP_LOGE(TAG, "Failed to create decel_motor_encoder");
        return;
    }

    // ESP_LOGI(TAG, "Enable RMT channel");
    // Debug: print motor_chan handle before enabling
    // ESP_LOGI(TAG, "motor_chan handle: %p", motor_chan);
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
        // Read physical inputs first
        int physical_jog_up = gpio_get_level(JOG_UP_GPIO);
        int physical_jog_down = gpio_get_level(JOG_DOWN_GPIO);
        int limit_switch = gpio_get_level(LIMIT_SWITCH_GPIO); // 1 = OK, 0 = limit hit
        int physical_start_cut = gpio_get_level(START_CUT_GPIO); // 1 = start, 0 = stop
        
        // Initialize control variables
        int jog_up = 0;
        int jog_down = 0;
        int start_cut = 0;
        
        // Determine control source based on web_control_active flag
        if (web_control_active) {
            // Web interface has control
            if (get_jog_up_requested()) {
                ESP_LOGI(TAG, "Executing jog up from web interface");
                jog_up = 1;
            }
            
            if (get_jog_down_requested()) {
                ESP_LOGI(TAG, "Executing jog down from web interface");
                jog_down = 1;
            }
            
            // Web interface controls EDM cutting
            if (edm_cutting_enabled) {
                start_cut = 1;
                static bool log_web_start = true;
                if (log_web_start) {
                    ESP_LOGI(TAG, "EDM cutting enabled via web interface");
                    log_web_start = false;
                }
            } else {
                start_cut = 0;
                static bool log_web_stop = true;
                if (log_web_stop && edm_cutting_enabled != 0) {
                    ESP_LOGI(TAG, "EDM cutting stopped via web interface");
                    log_web_stop = false;
                }
            }
        } else {
            // Physical buttons have control (default)
            jog_up = physical_jog_up;
            jog_down = physical_jog_down;
            start_cut = physical_start_cut;
        }

      
        // If limit switch is OFF, inhibit all movement
        if (!limit_switch) {
            // ESP_LOGI(TAG, "Limit switch hit, stopping all movement");
            // Stop the encoder if running
            if (encoder_running) {
                rmt_disable(motor_chan);
                encoder_running = false;
            }
            vTaskDelay(pdMS_TO_TICKS(20)); // Yield to avoid WDT and CPU hogging
            continue;
        }


             // if limit switch is OFF, inhibit Jog UP for now
        if (jog_up) {
            // ESP_LOGI(TAG, "Jog UP pressed");
            jogging = 1;
            gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_COUNTERCLOCKWISE); // Retract direction
            uint32_t steps = 1; // Large burst for smooth jog
            // ESP_LOGI(TAG, "Jog UP: accel phase");
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &steps, sizeof(steps), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            encoder_running = true;
            // Keep jogging at constant speed while button is held
            // Check control mode to determine how to detect button release
            if (web_control_active) {
                // For web control, jog for a short burst only (web buttons are momentary)
                for (int i = 0; i < 10 && gpio_get_level(LIMIT_SWITCH_GPIO); i++) {
                    steps = 1;
                    ESP_ERROR_CHECK(rmt_transmit(motor_chan, jog_motor_encoder, &steps, sizeof(steps), &tx_config));
                    ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
                    vTaskDelay(pdMS_TO_TICKS(10)); // Add delay to avoid WDT
                }
            } else {
                // For physical buttons, continue while button is held
                while (gpio_get_level(JOG_UP_GPIO) && gpio_get_level(LIMIT_SWITCH_GPIO)) {
                    //ESP_LOGI(TAG, "Jog UP: uniform phase");
                    steps = 1;
                    ESP_ERROR_CHECK(rmt_transmit(motor_chan, jog_motor_encoder, &steps, sizeof(steps), &tx_config));
                    ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
                    vTaskDelay(pdMS_TO_TICKS(5)); // Add small delay to avoid WDT
                }
            }
            // ESP_LOGI(TAG, "Jog UP: decel phase");
            steps = 1;
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &steps, sizeof(steps), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            jogging = 0;
        } else if (jog_down) {
            // ESP_LOGI(TAG, "Jog DOWN pressed");
            jogging = -1;
            gpio_set_level(STEP_MOTOR_GPIO_DIR, STEP_MOTOR_SPIN_DIR_CLOCKWISE);
            uint32_t steps = 1; // Large burst for smooth jog
            // ESP_LOGI(TAG, "Jog DOWN: accel phase");
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, accel_motor_encoder, &steps, sizeof(steps), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            encoder_running = true;
            // Keep jogging at constant speed while button is held
            // Check control mode to determine how to detect button release
            if (web_control_active) {
                // For web control, jog for a short burst only (web buttons are momentary)
                for (int i = 0; i < 10 && gpio_get_level(LIMIT_SWITCH_GPIO); i++) {
                    steps = 1;
                    ESP_ERROR_CHECK(rmt_transmit(motor_chan, jog_motor_encoder, &steps, sizeof(steps), &tx_config));
                    ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
                    vTaskDelay(pdMS_TO_TICKS(10)); // Add delay to avoid WDT
                }
            } else {
                // For physical buttons, continue while button is held
                while (gpio_get_level(JOG_DOWN_GPIO) && gpio_get_level(LIMIT_SWITCH_GPIO)) {
                    //ESP_LOGI(TAG, "Jog DOWN: uniform phase");
                    steps = 1;
                    ESP_ERROR_CHECK(rmt_transmit(motor_chan, jog_motor_encoder, &steps, sizeof(steps), &tx_config));
                    ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
                    vTaskDelay(pdMS_TO_TICKS(5)); // Add small delay to avoid WDT
                }
            }
            // ESP_LOGI(TAG, "Jog DOWN: decel phase");
            steps = 1;
            ESP_ERROR_CHECK(rmt_transmit(motor_chan, decel_motor_encoder, &steps, sizeof(steps), &tx_config));
            ESP_ERROR_CHECK(rmt_tx_wait_all_done(motor_chan, -1));
            encoder_running = true;
            jogging = 0;
        } else if (!jogging && limit_switch && start_cut) {
            // ESP_LOGI(TAG, "Start EDM cut");
            // int delay_ticks = 0; // Removed: delay_ticks no longer used
            int gap_voltage = adc_value_on_capture;
            
            // Check if ADC value is valid
            if (gap_voltage == -1) {
                // ESP_LOGW(TAG, "EDM: Invalid ADC value (-1), skipping control");
                vTaskDelay(pdMS_TO_TICKS(20));
                continue;
            }
            
            // Debug logging for ADC values
            static uint32_t last_debug_time = 0;
            uint32_t current_time = xTaskGetTickCount();
            if (current_time - last_debug_time > pdMS_TO_TICKS(2000)) { // Log every 2 seconds
                ESP_LOGI(TAG, "EDM DEBUG: gap_voltage=%d, LOW_VOLTAGE=%d, HIGH_VOLTAGE=%d, limit_switch=%d, start_cut=%d", 
                         gap_voltage, LOW_VOLTAGE, HIGH_VOLTAGE, limit_switch, start_cut);
                last_debug_time = current_time;
            }
            
            // Control logic
            int step_direction = 0;
            if (gap_voltage < LOW_VOLTAGE) {
                step_direction = -1;
                // ESP_LOGI(TAG, "EDM: Too close (ADC=%d < %d), retracting electrode", gap_voltage, LOW_VOLTAGE);
            } else if (gap_voltage > HIGH_VOLTAGE) {
                step_direction = 1;
                // ESP_LOGI(TAG, "EDM: Too far (ADC=%d > %d), advancing electrode", gap_voltage, HIGH_VOLTAGE);
            } else {
                step_direction = 0;
                // ESP_LOGI(TAG, "EDM: Gap OK (ADC=%d), holding position", gap_voltage);
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
        } else {
            // If no action is taken, ensure we still yield to avoid WDT
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        
        // Guarantee minimum delay in all paths to prevent watchdog timeout
        vTaskDelay(pdMS_TO_TICKS(1));
    }

}

void app_main(void)
{
    pwm_adc_queue = xQueueCreate(1, sizeof(int));
    
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
    ESP_LOGI(TAG, "NVS initialized successfully");
    
    // Initialize settings system
    esp_err_t settings_ret = settings_init();
    if (settings_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize settings system");
    } else {
        // Load settings and apply to global variables
        edm_settings_t settings;
        if (settings_load(&settings) == ESP_OK) {
            duty_percent = settings.duty_percent;
            adc_blanking_delay_ticks = settings.adc_blanking_delay;
            pid_kp = settings.pid_kp;
            pid_ki = settings.pid_ki;
            pid_kd = settings.pid_kd;
            pid_setpoint = settings.pid_setpoint;
            pid_control_enabled = settings.pid_control_enabled;
            ESP_LOGI(TAG, "Settings loaded: Duty=%d%%, ADC Delay=%d ticks", 
                     settings.duty_percent, settings.adc_blanking_delay);
            ESP_LOGI(TAG, "PID: Kp=%.3f, Ki=%.3f, Kd=%.3f, Setpoint=%d, Enabled=%s",
                     settings.pid_kp, settings.pid_ki, settings.pid_kd, settings.pid_setpoint,
                     settings.pid_control_enabled ? "true" : "false");
        }
    }
    
    // Create the main EDM tasks
    xTaskCreate(stepper_task, "stepper_task", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "Stepper motor example started");

    adc_oneshot_init(); // Initialize ADC before starting ADC task
    xTaskCreate(adc_on_capture_task, "adc_on_capture_task", 4096, NULL, 1, NULL); // Very low priority to prevent watchdog timeout

    // Start MCPWM task for power train
    xTaskCreate(mcpwm_halfbridge_task, "mcpwm_halfbridge_task", 4096, NULL, 5, NULL);
    
    // Start web server integration task
    xTaskCreate(web_integration_task, "web_integration_task", 8192, NULL, 2, NULL);
    ESP_LOGI(TAG, "Web integration task started");
    
    // The web_integration_task handles WiFi and web server initialization
    ESP_LOGI(TAG, "WiFi and web server will be started by web_integration_task");
}
