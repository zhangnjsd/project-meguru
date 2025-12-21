#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/ledc.h"
#include "driver/gpio.h"

#define TAG "SERVO_RESET"

#define SERVO_FREQ_HZ 50                          /*!< Servo Frequency 50Hz */
#define SERVO_PERIOD_US (1000000 / SERVO_FREQ_HZ) /*!< Servo Period in microseconds */

#define SERVO_LIFT_A_PIN GPIO_NUM_35 /*!< Lifting Arm A Servo Pin */
#define SERVO_LIFT_B_PIN GPIO_NUM_36 /*!< Lifting Arm B Servo Pin */
#define SERVO_LIFT_C_PIN GPIO_NUM_37 /*!< Lifting Arm C Servo Pin */
#define SERVO_CLAW_PIN GPIO_NUM_4    /*!< Mechanical Claw Servo Pin */

#define SERVO_MIN_US 500 /*!< Minimum pulse width (0 position) */
#define SERVO_MAX_US 2500 /*!< Maximum pulse width (Full position) */

// Helper function to calculate duty cycle from pulse width in microseconds
static uint16_t servo_duty_from_pulse_us(uint32_t pulse_us)
{
    // Duty cycle = (pulse_width / period) * (2^resolution - 1)
    // Resolution is 12-bit (4096 levels)
    // Period is 20000us (50Hz)
    return (uint16_t)((pulse_us * 4095) / SERVO_PERIOD_US);
}

void app_main(void)
{
    ESP_LOGI(TAG, "Servo Reset Test Started");

    // * Dedicated servo timer configuration
    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&servo_timer));

    // * PWM Channel Configuration for Lifting Arm A
    ledc_channel_config_t arm_lift_a_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_4,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_LIFT_A_PIN,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&arm_lift_a_channel));

    // * PWM Channel Configuration for Mechanical Claw
    ledc_channel_config_t claw_sw_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_5,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_CLAW_PIN,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&claw_sw_channel));

    // * PWM Channel Configuration for Lifting Arm B
    ledc_channel_config_t arm_lift_b_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_6,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_LIFT_B_PIN,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&arm_lift_b_channel));

    // * PWM Channel Configuration for Lifting Arm C
    ledc_channel_config_t arm_lift_c_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_7,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_LIFT_C_PIN,
        .duty = 0,
        .hpoint = 0,
    };
    ESP_ERROR_CHECK(ledc_channel_config(&arm_lift_c_channel));

    ESP_LOGI(TAG, "Starting Servo Sweep Test (5s Up, 5s Down)");

    while (1)
    {
        // Sweep Up: 0% -> 100% over 5 seconds
        ESP_LOGI(TAG, "Sweeping UP to MAX (2500us)...");
        for (int i = 0; i <= 100; i++)
        {
            uint32_t pulse = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * i / 100;
            uint16_t duty = servo_duty_from_pulse_us(pulse);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7);

            vTaskDelay(pdMS_TO_TICKS(50)); // 50ms * 100 steps = 5000ms = 5s
        }

        // Sweep Down: 100% -> 0% over 5 seconds
        ESP_LOGI(TAG, "Sweeping DOWN to MIN (500us)...");
        for (int i = 0; i <= 100; i++)
        {
            uint32_t pulse = SERVO_MAX_US - (SERVO_MAX_US - SERVO_MIN_US) * i / 100;
            uint16_t duty = servo_duty_from_pulse_us(pulse);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6);

            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7, duty);
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_7);

            vTaskDelay(pdMS_TO_TICKS(50)); // 50ms * 100 steps = 5000ms = 5s
        }
    }
}

