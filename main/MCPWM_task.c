#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/semphr.h"
#include "esp_rom_sys.h"
#include "esp_timer.h"

#define MCPWM_GPIO_PWM0A   16
#define MCPWM_GPIO_PWM0B   17
#define MCPWM_CAP_GPIO   18  // External signal capture pin
#define PWM_FREQ_HZ        20000
#define DEAD_TIME_NS       100

volatile int duty_percent = 40; // Start with 40%, change this variable from elsewhere
volatile uint32_t last_pwm_rising_ticks = 0; // Timestamp of last PWM rising edge
volatile uint32_t last_capture_ticks = 0;
volatile uint32_t delay_ticks = 0; // Store the interval between PWM and capture
SemaphoreHandle_t capture_semaphore = NULL;
volatile bool pid_control_enabled = false; // Flag to enable/disable PID control

// PID control variables - now configurable from web interface
volatile float pid_kp = 0.05f;   // Proportional gain
volatile float pid_ki = 0.01f;   // Integral gain
volatile float pid_kd = 0.0f;    // Derivative gain
volatile int pid_setpoint = 1500; // Target ADC value

extern volatile int adc_value_on_capture;

// Callback for PWM rising edge (GPIO 16) - using timer event instead of generator event
static bool IRAM_ATTR pwm_rising_cb(mcpwm_timer_handle_t timer, const mcpwm_timer_event_data_t *edata, void *user_data)
{
    // Record the timer value at PWM rising edge (timer empty event)
    // For ESP-IDF v5.4.1, we'll use a simple timestamp approach
    // The exact timing isn't critical for this application
    last_pwm_rising_ticks = xTaskGetTickCountFromISR() * portTICK_PERIOD_MS * 10000; // Convert to 0.1us ticks
    return false;
}

static bool IRAM_ATTR capture_cb(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    // Calculate propagation delay between PWM rising edge and capture event
    delay_ticks = (edata->cap_value >= last_pwm_rising_ticks) ?
        (edata->cap_value - last_pwm_rising_ticks) :
        (0xFFFFFFFF - last_pwm_rising_ticks + edata->cap_value + 1);
    // Only trigger the ADC task (e.g., by giving the semaphore)
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    if (capture_semaphore) {
        xSemaphoreGiveFromISR(capture_semaphore, &xHigherPriorityTaskWoken);
    }
    if (xHigherPriorityTaskWoken) {
        portYIELD_FROM_ISR();
    }
    return false;
}

void setup_mcpwm_capture(mcpwm_timer_handle_t timer)
{
    mcpwm_cap_timer_handle_t cap_timer = NULL;
    mcpwm_capture_timer_config_t cap_timer_config = {
        .group_id = 0,
        .resolution_hz = 10000000, // 0.1us per tick
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_timer(&cap_timer_config, &cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_enable(cap_timer));
    ESP_ERROR_CHECK(mcpwm_capture_timer_start(cap_timer));

    mcpwm_cap_channel_handle_t cap_chan = NULL;
    mcpwm_capture_channel_config_t cap_chan_config = {
        .gpio_num = MCPWM_CAP_GPIO,
        .prescale = 1,
        .flags.neg_edge = false,
        .flags.pos_edge = true, // capture on rising edge
        .flags.pull_up = true,
    };
    ESP_ERROR_CHECK(mcpwm_new_capture_channel(cap_timer, &cap_chan_config, &cap_chan));
    ESP_ERROR_CHECK(mcpwm_capture_channel_register_event_callbacks(cap_chan, &(mcpwm_capture_event_callbacks_t){
        .on_cap = capture_cb,
    }, NULL));
    ESP_ERROR_CHECK(mcpwm_capture_channel_enable(cap_chan));
}

void mcpwm_halfbridge_task(void *pvParameters)
{
    mcpwm_timer_handle_t timer = NULL;
    mcpwm_timer_config_t timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000,
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
        .period_ticks = 10000000 / PWM_FREQ_HZ,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&timer_config, &timer));

    mcpwm_oper_handle_t oper = NULL;
    mcpwm_operator_config_t operator_config = { .group_id = 0 };
    ESP_ERROR_CHECK(mcpwm_new_operator(&operator_config, &oper));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(oper, timer));

    mcpwm_cmpr_handle_t comparator = NULL;
    mcpwm_comparator_config_t comparator_config = { .flags.update_cmp_on_tez = true };
    ESP_ERROR_CHECK(mcpwm_new_comparator(oper, &comparator_config, &comparator));

    mcpwm_gen_handle_t gen_a = NULL, gen_b = NULL;
    mcpwm_generator_config_t gen_config_a = { .gen_gpio_num = MCPWM_GPIO_PWM0A };
    mcpwm_generator_config_t gen_config_b = { .gen_gpio_num = MCPWM_GPIO_PWM0B };
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config_a, &gen_a));
    ESP_ERROR_CHECK(mcpwm_new_generator(oper, &gen_config_b, &gen_b));

    // Register timer event callback for PWM rising edge detection
    ESP_ERROR_CHECK(mcpwm_timer_register_event_callbacks(timer, &(mcpwm_timer_event_callbacks_t){
        .on_empty = pwm_rising_cb,
    }, NULL));

    mcpwm_dead_time_config_t dt_config = {
        .posedge_delay_ticks = DEAD_TIME_NS / 100,
        .negedge_delay_ticks = DEAD_TIME_NS / 100,
        .flags = { .invert_output = 0 }
    };
    // Set dead time on generator B (complementary output)
    ESP_ERROR_CHECK(mcpwm_generator_set_dead_time(gen_b, gen_a, &dt_config));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen_a, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen_a, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(
        gen_b, MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_LOW)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(
        gen_b, MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_HIGH)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP));

    // Create semaphore for capture event
    capture_semaphore = xSemaphoreCreateBinary();
    // Setup capture for external signal
    setup_mcpwm_capture(timer);

    uint32_t period_ticks = timer_config.period_ticks;

    // Soft start: ramp duty from 0 to current duty_percent
    int target_duty = duty_percent;
    for (int soft_duty = 0; soft_duty <= target_duty; soft_duty++) {
        uint32_t duty_ticks = period_ticks * soft_duty / 100;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty_ticks));
        vTaskDelay(pdMS_TO_TICKS(20)); // Adjust delay for ramp speed
    }
    // duty_percent is already set, don't override it
    ESP_LOGI("MCPWM", "MCPWM task started with duty_percent=%d%%, pid_control_enabled=%s", 
             duty_percent, pid_control_enabled ? "true" : "false");

    // PID control variables
    static double pid_integral = 0;
    static double pid_prev_error = 0;
    const double DUTY_MIN = 1.0;
    const double DUTY_MAX = 99.0;

    while (1) {
        // Clamp duty_percent to [DUTY_MIN, DUTY_MAX]
        int local_duty = duty_percent;
        if (local_duty < DUTY_MIN) local_duty = DUTY_MIN;
        if (local_duty > DUTY_MAX) local_duty = DUTY_MAX;
        
        // Debug: log duty cycle changes (but not too frequently)
        static int last_logged_duty = -1;
        static uint32_t last_log_time = 0;
        uint32_t current_time = xTaskGetTickCount();
        if (local_duty != last_logged_duty && (current_time - last_log_time) > pdMS_TO_TICKS(5000)) {
            ESP_LOGI("MCPWM", "Duty cycle: %d%% (PID: %s, ADC: %d)", local_duty, 
                     pid_control_enabled ? "enabled" : "disabled", adc_value_on_capture);
            last_logged_duty = local_duty;
            last_log_time = current_time;
        }
        
        // Special check for duty cycle being reset to 40%
        static int last_duty_percent = -1;
        if (duty_percent != last_duty_percent) {
            if (duty_percent == 40 && last_duty_percent != -1) {
                ESP_LOGW("MCPWM", "WARNING: Duty cycle reset to 40%% (was %d%%)", last_duty_percent);
            }
            last_duty_percent = duty_percent;
        }

        // --- PID control for duty_percent ---
        static bool last_pid_enabled = false;
        if (pid_control_enabled != last_pid_enabled) {
            ESP_LOGI("MCPWM", "PID control %s", pid_control_enabled ? "enabled" : "disabled");
            last_pid_enabled = pid_control_enabled;
        }
        
        if (pid_control_enabled) {
            int adc_value = adc_value_on_capture;
            if (adc_value >= 0) {
                double error = pid_setpoint - adc_value;
                pid_integral += error;
                double derivative = error - pid_prev_error;
                double pid_output = pid_kp * error + pid_ki * pid_integral + pid_kd * derivative;
                pid_prev_error = error;
                int old_duty = duty_percent;
                duty_percent += pid_output;
                if (duty_percent < DUTY_MIN) duty_percent = DUTY_MIN;
                if (duty_percent > DUTY_MAX) duty_percent = DUTY_MAX;
                if (old_duty != duty_percent) {
                    ESP_LOGI("MCPWM", "PID adjusted duty: %d -> %d (ADC: %d, error: %.1f, output: %.2f)", 
                             old_duty, duty_percent, adc_value, error, pid_output);
                }
            } else {
                ESP_LOGW("MCPWM", "PID enabled but no ADC value available");
            }
        }
        // --- End PID control ---

        uint32_t duty_ticks = period_ticks * local_duty / 100;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty_ticks));

        // Small delay to allow current to propagate after high side turns on
       // esp_rom_delay_us(2); // Adjust as needed for your hardware  //delay was added in ADC.c

        // Trigger ADC task via semaphore (like before)
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (capture_semaphore) {
            xSemaphoreGive(capture_semaphore);
        }

        vTaskDelay(pdMS_TO_TICKS(20)); // Update rate
    }
}
