#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "includes/i2c9555.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#ifndef TAG
#define TAG "MOTOR_TEST"
#endif

#define SDA_PIN GPIO_NUM_8 /*!< SDA BUS*/
#define SCL_PIN GPIO_NUM_9 /*!< SCL BUS*/

#define MTR_FL_PWM GPIO_NUM_10 /*!< Front-Left Motor*/
#define MTR_FR_PWM GPIO_NUM_11 /*!< Front-Right Motor*/
#define MTR_BL_PWM GPIO_NUM_12 /*!< Back-Left Motor*/
#define MTR_BR_PWM GPIO_NUM_13 /*!< Back-Right Motor*/

#define MTR_FULL_SPD 4096 /*!< Full Speed Duty Cycle */
#define MTR_STOP 0        /*!< Stop Duty Cycle */

int device_mtr;

// ? Diagnostic function to check I2C connectivity
static void i2c_diagnostic()
{
    ESP_LOGI(TAG, "=== I2C Diagnostic Start ===");
    ESP_LOGI(TAG, "SDA Pin: GPIO_%d", TCA_SDA_PIN);
    ESP_LOGI(TAG, "SCL Pin: GPIO_%d", SCL_PIN);
    ESP_LOGI(TAG, "I2C9555 Address: 0x20");
    ESP_LOGI(TAG, "Motor PWM Pins: FL=%d, FR=%d, BL=%d, BR=%d", 
             MTR_FL_PWM, MTR_FR_PWM, MTR_BL_PWM, MTR_BR_PWM);
    ESP_LOGI(TAG, "=== I2C Diagnostic End ===");
}

// ? Motor Full Speed Forward - All IN1=1, IN2=0
static void motor_full_speed_forward()
{
    ESP_LOGI(TAG, "Motor Full Speed Forward: ALL IN1=1, IN2=0, PWM=FULL");
    
    // Set motor directions: IN1=1, IN2=0 for all motors
    i2c9555pin_write(device_mtr, EXT_IO0, 1); // * MTR_FL_IN1 = 1
    i2c9555pin_write(device_mtr, EXT_IO1, 0); // * MTR_FL_IN2 = 0
    i2c9555pin_write(device_mtr, EXT_IO2, 1); // * MTR_FR_IN1 = 1
    i2c9555pin_write(device_mtr, EXT_IO3, 0); // * MTR_FR_IN2 = 0
    i2c9555pin_write(device_mtr, EXT_IO4, 1); // * MTR_BL_IN1 = 1
    i2c9555pin_write(device_mtr, EXT_IO5, 0); // * MTR_BL_IN2 = 0
    i2c9555pin_write(device_mtr, EXT_IO6, 1); // * MTR_BR_IN1 = 1
    i2c9555pin_write(device_mtr, EXT_IO7, 0); // * MTR_BR_IN2 = 0
    
    // Set all motor speeds to full speed
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MTR_FULL_SPD); // * Front-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, MTR_FULL_SPD); // * Front-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, MTR_FULL_SPD); // * Back-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, MTR_FULL_SPD); // * Back-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

// ? Stop All Motors
static void motor_stop()
{
    ESP_LOGI(TAG, "Stopping all motors");
    
    // Set motor speeds to zero
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MTR_STOP); // * Front-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, MTR_STOP); // * Front-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, MTR_STOP); // * Back-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, MTR_STOP); // * Back-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Motor Test Application Started");
    
    // Run diagnostic
    i2c_diagnostic();
    
    // Wait a bit for system stabilization
    vTaskDelay(pdMS_TO_TICKS(500));
    
    // * Initialize Motor Controller via 9555
    ESP_LOGI(TAG, "Adding I2C9555 device at address 0x40 on SDA=%d, SCL=%d", TCA_SDA_PIN, SCL_PIN);
    device_mtr = i2c9555_add_device(TCA_SDA_PIN, SCL_PIN, 0x20, GPIO_NUM_NC, NULL);
    
    if (device_mtr < 0)
    {
        ESP_LOGE(TAG, "Failed to add I2C9555 device! device_mtr = %d", device_mtr);
        ESP_LOGE(TAG, "Possible reasons:");
        ESP_LOGE(TAG, "1. I2C9555 device not responding at address 0x40");
        ESP_LOGE(TAG, "2. Check SDA (GPIO_%d) and SCL (GPIO_%d) connections", TCA_SDA_PIN, SCL_PIN);
        ESP_LOGE(TAG, "3. Verify pull-up resistors on I2C bus");
        ESP_LOGE(TAG, "4. Check power supply to I2C9555");
        
        // Blink error - just loop
        while (1)
        {
            ESP_LOGE(TAG, "Waiting for I2C device...");
            vTaskDelay(pdMS_TO_TICKS(1000));
        }
    }
    
    ESP_LOGI(TAG, "I2C9555 device added successfully, device_id=%d", device_mtr);
    vTaskDelay(pdMS_TO_TICKS(100));
    
    /*  Motor Controller Configuration
        Pin     Functions       State
        00      MTR_FL_IN1      Output
        01      MTR_FL_IN2      Output
        02      MTR_FR_IN1      Output
        03      MTR_FR_IN2      Output
        04      MTR_BL_IN1      Output
        05      MTR_BL_IN2      Output
        06      MTR_BR_IN1      Output
        07      MTR_BR_IN2      Output
    */
    ESP_LOGI(TAG, "Configuring I2C9555 IO as all outputs (0x0000)");
    i2c9555_ioconfig(device_mtr, 0x0000);
    ESP_LOGI(TAG, "Motor controller I/O configured");
    vTaskDelay(pdMS_TO_TICKS(100));

    // * PWM Initialization
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_12_BIT, // 4096 levels
        .freq_hz = 5000,                   // 5 kHz frequency
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&pwm_timer);
    ESP_LOGI(TAG, "PWM timer configured");

    // * PWM Channel Configuration for all four motors
    ledc_channel_config_t mtr_fl_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_FL_PWM,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config_t mtr_fr_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_FR_PWM,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config_t mtr_bl_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_BL_PWM,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config_t mtr_br_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_3,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_BR_PWM,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&mtr_fl_channel);
    ledc_channel_config(&mtr_fr_channel);
    ledc_channel_config(&mtr_bl_channel);
    ledc_channel_config(&mtr_br_channel);
    ESP_LOGI(TAG, "All PWM channels configured");

    // * Test sequence
    ESP_LOGI(TAG, "Starting motor test sequence...");
    vTaskDelay(pdMS_TO_TICKS(1000)); // Wait 1 second before starting
    
    // Run motors at full speed
    motor_full_speed_forward();
    ESP_LOGI(TAG, "Motors running at full speed - IN1=1, IN2=0, PWM=8192");
    
    // Keep running indefinitely
    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(100));
    }
}
