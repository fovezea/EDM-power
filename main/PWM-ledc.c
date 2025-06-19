#include "driver/ledc.h"
#include "esp_rom_sys.h" // For ets_delay_us
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "esp_adc/adc_oneshot.h"
//#include "esp_adc_cal.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "soc/soc_caps.h"
// Define your half-bridge pins and dead time (in microseconds)
//#define HALFBRIDGE_HIGH_GPIO   16   //originally 16
//#define HALFBRIDGE_LOW_GPIO    17
#define HALFBRIDGE_HIGH_GPIO   17
#define HALFBRIDGE_LOW_GPIO    16
#define PWM_FREQ_HZ            20000
#define PWM_RES_BITS           LEDC_TIMER_10_BIT
#define DEAD_TIME_US           2   // 2us dead time (adjust as needed)
#define ADC_READ_DELAY_MS 10 // Delay for ADC reading in milliseconds

static const char *TAG = "halfbridge_pwm";
// Global queue to hold ADC readings
QueueHandle_t pwm_adc_queue;
#define ADC_ATTEN           ADC_ATTEN_DB_12
#define ADC_CHANNEL    ADC_CHANNEL_3 

adc_oneshot_unit_handle_t adc_handle; // ADC handle
//esp_adc_cal_characteristics_t adc_chars;
bool adc_calibration_init(adc_unit_t unit, adc_channel_t channel, adc_atten_t atten, adc_cali_handle_t *out_handle);


void halfbridge_pwm_task(void *pvParameters)
{

 
    
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
    ESP_ERROR_CHECK(adc_oneshot_config_channel(adc_handle, ADC_CHANNEL, &chan_cfg));

    // --- ADC CALIBRATION ---
   // ESP_LOGI(TAG, "ADC calibration");
   // esp_adc_cal_characteristics_t adc_chars_local;
    //esp_adc_cal_characterize(ADC_UNIT_1, ADC_ATTEN_DB_12, ADC_WIDTH_BIT_12, 1100, &adc_chars_local);
    //adc_chars = adc_chars_local; 
   // adc_cali_handle_t adc1_cali_chan6_handle = NULL;
  //  bool do_calibration1_chan6 = adc_calibration_init(ADC_UNIT_1, ADC_CHANNEL_6, ADC_ATTEN, &adc1_cali_chan6_handle);



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
    uint32_t dead_ticks = (PWM_FREQ_HZ * DEAD_TIME_US) / 1000000 * max_duty / PWM_FREQ_HZ;

   
    int adc_reading = 0; // Variable to hold ADC reading
    //int voltage = 0; // Variable to hold calibrated voltage reading
    
    // Main loop for PWM control
     while (1) {

       // if (do_calibration1_chan6) {
        //    ESP_ERROR_CHECK(adc_cali_raw_to_voltage(adc1_cali_chan6_handle, adc_reading, &voltage));
        //    ESP_LOGI(TAG, "ADC%d Channel[%d] Cali Voltage: %d mV", ADC_UNIT_1 + 1, ADC_CHANNEL_6, voltage);
           // xQueueReceive(pwm_adc_queue, &pwm_adc_value, pdMS_TO_TICKS(10));

       // This is stopping the PWM if ADC reading is above a threshold 
       //but probably is better not to stop it, just reduce the duty cycle or do nothing at all.
       //also it is wrong as the ADC is stoped and the value is not updated and the watch dog will trigger.
       // if (adc_reading > 3500) {
       //     ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_0, 0);
       //     ledc_stop(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
       //     continue;//this is not good here.
       // }

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
        //esp_rom_delay_us(ADC_READ_DELAY_MS);
        // Read ADC value
        // esp_err_t err = adc_oneshot_read(adc_handle, ADC_CHANNEL, &adc_reading);
       // if (err != ESP_OK) {
        //    ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(err));
        //}
        //update the queue with the ADC reading
       // xQueueOverwrite(pwm_adc_queue, &adc_reading); // Overwrite with latest value
         
       // esp_rom_delay_us((1000000 * low_on_percent / 100 / PWM_FREQ_HZ) - DEAD_TIME_US - ADC_READ_DELAY_MS);
       esp_rom_delay_us((1000000 * low_on_percent / 100 / PWM_FREQ_HZ) - DEAD_TIME_US);
        // Both OFF for dead time
        ledc_set_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1, 0);
        ledc_update_duty(LEDC_HIGH_SPEED_MODE, LEDC_CHANNEL_1);
        esp_rom_delay_us(DEAD_TIME_US);
        vTaskDelay(pdMS_TO_TICKS(10));
    }

    }

/*---------------------------------------------------------------
        ADC Calibration
---------------------------------------------------------------*/
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

  static void adc_calibration_deinit(adc_cali_handle_t handle)
{
#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Curve Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_curve_fitting(handle));

#elif ADC_CALI_SCHEME_LINE_FITTING_SUPPORTED
    ESP_LOGI(TAG, "deregister %s calibration scheme", "Line Fitting");
    ESP_ERROR_CHECK(adc_cali_delete_scheme_line_fitting(handle));
#endif
} 