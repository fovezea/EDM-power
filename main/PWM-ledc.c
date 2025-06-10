#include "driver/ledc.h"
#include "esp_rom_sys.h" // For ets_delay_us
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc_cal.h"


// Define your half-bridge pins and dead time (in microseconds)
#define HALFBRIDGE_HIGH_GPIO   16
#define HALFBRIDGE_LOW_GPIO    17
#define PWM_FREQ_HZ            20000
#define PWM_RES_BITS           LEDC_TIMER_10_BIT
#define DEAD_TIME_US           2   // 2us dead time (adjust as needed)
#define ADC_READ_DELAY_MS 10 // Delay for ADC reading in milliseconds

static const char *TAG = "halfbridge_pwm";
//extern volatile int g_pwm_adc_value ; // Global variable for ADC value but replaced with queue
QueueHandle_t pwm_adc_queue;
//pwm_adc_queue = xQueueCreate(4, sizeof(int));

adc_oneshot_unit_handle_t adc_handle; // Add this global
esp_adc_cal_characteristics_t adc_chars;



void halfbridge_pwm_task(void *pvParameters)
{

 //pwm_adc_queue = xQueueCreate(4, sizeof(int));
 // pwm_adc_queue = xQueueCreate(1, sizeof(int));  
    
  // --- ADC ONESHOT INIT ---
    ESP_LOGI(TAG, "Initialize ADC1 channel 6 (GPIO34) with new API");
    adc_oneshot_unit_init_cfg_t init_config = {
        .unit_id = ADC_UNIT_1,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_config, &adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL_6, &chan_cfg));

    // --- ADC CALIBRATION ---
    ESP_LOGI(TAG, "ADC calibration");
    esp_adc_cal_characteristics_t adc_chars_local;
    esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars_local);
    adc_chars = adc_chars_local; // copy to global for use in task






    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_HIGH_SPEED_MODE,
        .timer_num        = LEDC_TIMER_0,
        .duty_resolution  = PWM_RES_BITS,
        .freq_hz          = PWM_FREQ_HZ,
        .clk_cfg          = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    // Configure channel for HIGH side
    ledc_channel_config_t ledc_channel_high = {
        .gpio_num       = HALFBRIDGE_HIGH_GPIO,
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_0,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_high));

    // Configure channel for LOW side
    ledc_channel_config_t ledc_channel_low = {
        .gpio_num       = HALFBRIDGE_LOW_GPIO,
        .speed_mode     = LEDC_HIGH_SPEED_MODE,
        .channel        = LEDC_CHANNEL_1,
        .timer_sel      = LEDC_TIMER_0,
        .duty           = 0,
        .hpoint         = 0,
        .flags.output_invert = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&ledc_channel_low));

    // Calculate duty and dead time in ticks
   // uint32_t max_duty = (1 << PWM_RES_BITS) - 1;
   // uint32_t dead_ticks = (PWM_FREQ_HZ * DEAD_TIME_US) / 1000000 * max_duty / PWM_FREQ_HZ;
   // uint32_t main_duty = max_duty / 2 - dead_ticks; // 50% duty minus dead time


    // User-configurable ON times in percent (0-100)
    int high_on_percent = 40; // Example: 40% ON time for HIGH side
    int low_on_percent  = 40; // Example: 40% ON time for LOW side

    // Calculate duty and dead time in ticks
    uint32_t max_duty = (1 << PWM_RES_BITS) - 1;
    uint32_t high_on_ticks = (max_duty * high_on_percent) / 100;
    uint32_t low_on_ticks  = (max_duty * low_on_percent) / 100;
    //uint32_t dead_ticks = (PWM_FREQ_HZ * DEAD_TIME_US) / 1000000 * max_duty / PWM_FREQ_HZ;

   // int pwm_adc_value = 0;
    int adc_reading = 2048; // Variable to hold ADC reading
    
    // Main loop for PWM control
     while (1) {
       // xQueueReceive(pwm_adc_queue, &pwm_adc_value, pdMS_TO_TICKS(10));

        if (adc_reading > 3500) {
            ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
            ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
            continue;
        }

        // HIGH side ON, LOW side OFF
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, high_on_ticks);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        esp_rom_delay_us((1000000 * high_on_percent / 100 / PWM_FREQ_HZ) - DEAD_TIME_US);

        // Both OFF for dead time
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        esp_rom_delay_us(DEAD_TIME_US);

        // LOW side ON, HIGH side OFF
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, low_on_ticks);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0);
        // Add a short delay to allow the signal to stabilize (e.g., 10 microseconds)
        esp_rom_delay_us(ADC_READ_DELAY_MS);
         ESP_ERROR_CHECK(adc_oneshot_read(adc_handle, ADC_CHANNEL_6, &adc_reading));
        //update the queue with the ADC reading
        xQueueOverwrite(pwm_adc_queue, &adc_reading); // Overwrite with latest value
         
        esp_rom_delay_us((1000000 * low_on_percent / 100 / PWM_FREQ_HZ) - DEAD_TIME_US - ADC_READ_DELAY_MS);

        // Both OFF for dead time
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        esp_rom_delay_us(DEAD_TIME_US);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    }

   