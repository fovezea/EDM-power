#include "driver/mcpwm_prelude.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/semphr.h"

#define MCPWM_GPIO_PWM0A   16
#define MCPWM_GPIO_PWM0B   17
#define MCPWM_CAP_GPIO   18  // External signal capture pin
#define PWM_FREQ_HZ        20000
#define DEAD_TIME_NS       100

volatile int duty_percent = 40; // Start with 40%, change this variable from elsewhere
volatile uint32_t last_capture_ticks = 0;
SemaphoreHandle_t capture_semaphore = NULL;

static bool IRAM_ATTR capture_cb(mcpwm_cap_channel_handle_t cap_chan, const mcpwm_capture_event_data_t *edata, void *user_data)
{
    last_capture_ticks = edata->cap_value;
    static uint32_t capture_counter = 0;
    capture_counter++;
    if (capture_counter >= 100) { // Trigger every 100th event (adjust as needed)
        BaseType_t xHigherPriorityTaskWoken = pdFALSE;
        if (capture_semaphore) {
            xSemaphoreGiveFromISR(capture_semaphore, &xHigherPriorityTaskWoken);
        }
        if (xHigherPriorityTaskWoken) {
            portYIELD_FROM_ISR();
        }
        capture_counter = 0;
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

    // Soft start: ramp duty from 0 to 40%
    for (int soft_duty = 0; soft_duty <= 40; soft_duty++) {
        uint32_t duty_ticks = period_ticks * soft_duty / 100;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty_ticks));
        vTaskDelay(pdMS_TO_TICKS(20)); // Adjust delay for ramp speed
    }
    duty_percent = 40; // Ensure main loop starts at 40%

    while (1) {
        // Clamp duty_percent to [0, 100]
        int local_duty = duty_percent;
        if (local_duty < 0) local_duty = 0;
        if (local_duty > 100) local_duty = 100;

        uint32_t duty_ticks = period_ticks * local_duty / 100;
        ESP_ERROR_CHECK(mcpwm_comparator_set_compare_value(comparator, duty_ticks));

        vTaskDelay(pdMS_TO_TICKS(20)); // Update rate
    }
}
