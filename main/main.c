#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "includes/i2c9555.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"

#define TAG "MAIN"

#define SDA_PIN GPIO_NUM_8 /*!< SDA BUS*/
#define SCL_PIN GPIO_NUM_9 /*!< SDA BUS*/

#define MTR_FL_PWM GPIO_NUM_10 /*!< Front-Left Motor*/
#define MTR_FR_PWM GPIO_NUM_11 /*!< Front-Right Motor*/
#define MTR_BL_PWM GPIO_NUM_12 /*!< Back-Left Motor*/
#define MTR_BR_PWM GPIO_NUM_13 /*!< Back-Right Motor*/

#define MTR_FULL_SPD 8192
#define MTR_TURN_SPD 4096
#define MTR_SLOW_SPD 2048
#define MTR_STOP 0

int device_ir;
int device_mtr;

int bar_detected_flag = 0;

// ? Line-following State Machine
typedef enum
{
    NONE,              /*!<Priority: 0 No IR Sensor Detected Line*/
    BOTH_FORWARD,      /*!<Priority: 2 Both Forward Sensor Detected*/
    ONLY_L_FORWARD,    /*!<Priority: 1 Only Left Forward Sensor Detected*/
    ONLY_R_FORWARD,    /*!<Priority: 1 Only Right Forward Sensor Detected*/
    L_MED_DETECTED,    /*!<Priority: 4 Left Forward & L1 Medium Sensor Detected*/
    R_MED_DETECTED,    /*!<Priority: 4 Right Forward & R1 Medium Sensor Detected*/
    L_FIRST_DETECTED,  /*!<Priority: 3 L1 Sensor Detected*/
    R_FIRST_DETECTED,  /*!<Priority: 3 R1 Sensor Detected*/
    L_SECOND_DETECTED, /*!<Priority: 5 L2 Sensor Detected*/
    R_SECOND_DETECTED, /*!<Priority: 5 R2 Sensor Detected*/
    FULL_DETECTED,     /*!<Priority: 6 All Sensors Detected*/
} IRState;

// ? IR Data
typedef struct
{
    bool LL;
    bool LF;
    bool FL;
    bool FR;
    bool RF;
    bool RR;
} IRData;

volatile IRData current_ir_data;
volatile IRState current_ir_state = NONE;

// ! IR Module Input Callback
void ir_read_callback(uint16_t pin, int level);

// ! Motor Control based on IR State
void mtr_ctrl(void *args);

void app_main(void)
{

    current_ir_data = (IRData) {
        .LL = false,
        .LF = false,
        .FL = false,
        .FR = false,
        .RF = false,
        .RR = false,
    };

    // * Initialize IR_Module via 9555
    device_ir = i2c9555_add_device(SDA_PIN, SCL_PIN, 0x40, GPIO_NUM_39, ir_read_callback);
    /*  IR Module Pin Configuration
        Pin     Functions       State
        00      IR_LL           Input
        01      IR_LF           Input
        02      IR_FL           Input
        03      IR_FR           Input
        04      IR_RF           Input
        05      IR_RR           Input
    */
    i2c9555_ioconfig(device_ir, 0xFFFF);

    // * Initialize Motor Controller via 9555
    device_mtr = i2c9555_add_device(SDA_PIN, SCL_PIN, 0x42, GPIO_NUM_NC, NULL);
    /*  Motor Controller
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
    i2c9555_ioconfig(device_mtr, 0x0000);

    // * PWM Initialization
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_13_BIT,   // 8192 levels
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&pwm_timer);

    // * PWM Channel Configuration
    ledc_channel_config_t mtr_fl_channel = {
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .channel = LEDC_CHANNEL_0,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_FL_PWM,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config_t mtr_fr_channel = {
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .channel = LEDC_CHANNEL_1,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_FR_PWM,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config_t mtr_bl_channel = {
        .speed_mode = LEDC_SPEED_MODE_MAX,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_0,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = MTR_BL_PWM,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config_t mtr_br_channel = {
        .speed_mode = LEDC_SPEED_MODE_MAX,
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
    

    // * Motor Control Thread
    xTaskCreate(mtr_ctrl, "Motor Control Task", 2048, NULL, 4, NULL);
}

// ! IR Module Input Callback
void ir_read_callback(uint16_t pin, int level)
{
    ESP_LOGI(TAG, "IR Pin: 0x%04X Level: %d", pin, level);

    // * Update IR Data
    switch (pin)
    {
    case EXT_IO0:
        current_ir_data.LL = level;
        break;
    case EXT_IO1:
        current_ir_data.LF = level;
        break;
    case EXT_IO2:
        current_ir_data.FL = level;
        break;
    case EXT_IO3:
        current_ir_data.FR = level;
        break;
    case EXT_IO4:
        current_ir_data.RF = level;
        break;
    case EXT_IO5:
        current_ir_data.RR = level;
        break;
    default:
        break;
    }

    // * Update IR State Machine
    if (current_ir_data.RR)
    {
        current_ir_state = R_SECOND_DETECTED;
    }
    else if (current_ir_data.LL)
    {
        current_ir_state = L_SECOND_DETECTED;
    }
    else if (current_ir_data.RF && current_ir_data.FR)
    {
        current_ir_state = R_MED_DETECTED;
    }
    else if (current_ir_data.LF && current_ir_data.FL)
    {
        current_ir_state = L_MED_DETECTED;
    }
    else if (current_ir_data.RF)
    {
        current_ir_state = R_FIRST_DETECTED;
    }
    else if (current_ir_data.LF)
    {
        current_ir_state = L_FIRST_DETECTED;
    }
    else if (current_ir_data.FL && current_ir_data.FR)
    {
        current_ir_state = BOTH_FORWARD;
    }
    else if (current_ir_data.FR)
    {
        current_ir_state = ONLY_R_FORWARD;
    }
    else if (current_ir_data.FL)
    {
        current_ir_state = ONLY_L_FORWARD;
    }
    else
    {
        current_ir_state = NONE;
    }
}

// ? Motor Movement Control START
static void moveForwardFast()
{
    // Set motor directions for forward movement
    i2c9555pin_write(device_mtr, EXT_IO0, 1); // * MTR_FL_IN1
    i2c9555pin_write(device_mtr, EXT_IO1, 0); // * MTR_FL_IN2
    i2c9555pin_write(device_mtr, EXT_IO2, 1); // * MTR_FR_IN1
    i2c9555pin_write(device_mtr, EXT_IO3, 0); // * MTR_FR_IN2
    i2c9555pin_write(device_mtr, EXT_IO4, 1); // * MTR_BL_IN1
    i2c9555pin_write(device_mtr, EXT_IO5, 0); // * MTR_BL_IN2
    i2c9555pin_write(device_mtr, EXT_IO6, 1); // * MTR_BR_IN1
    i2c9555pin_write(device_mtr, EXT_IO7, 0); // * MTR_BR_IN2
    // Set motor speeds
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0, MTR_FULL_SPD);   // * Front-Left Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1, MTR_FULL_SPD);   // * Front-Right Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2, MTR_FULL_SPD);   // * Back-Left Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_3, MTR_FULL_SPD);   // * Back-Right Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_3);
}

static void sharpTurnLeft(float mag)
{
    // Set motor directions for left turn
    i2c9555pin_write(device_mtr, EXT_IO0, 0); // * MTR_FL_IN1
    i2c9555pin_write(device_mtr, EXT_IO1, 1); // * MTR_FL_IN2
    i2c9555pin_write(device_mtr, EXT_IO2, 1); // * MTR_FR_IN1
    i2c9555pin_write(device_mtr, EXT_IO3, 0); // * MTR_FR_IN2
    i2c9555pin_write(device_mtr, EXT_IO4, 0); // * MTR_BL_IN1
    i2c9555pin_write(device_mtr, EXT_IO5, 1); // * MTR_BL_IN2
    i2c9555pin_write(device_mtr, EXT_IO6, 1); // * MTR_BR_IN1
    i2c9555pin_write(device_mtr, EXT_IO7, 0); // * MTR_BR_IN2
    // Set motor speeds
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0, MTR_TURN_SPD * mag);   // * Front-Left Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1, MTR_TURN_SPD * mag);   // * Front-Right Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2, MTR_TURN_SPD * mag);   // * Back-Left Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_3, MTR_TURN_SPD * mag);   // * Back-Right Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_3);
}

static void sharpTurnRight(float mag)
{
    // Set motor directions for right turn
    i2c9555pin_write(device_mtr, EXT_IO0, 1); // * MTR_FL_IN1
    i2c9555pin_write(device_mtr, EXT_IO1, 0); // * MTR_FL_IN2
    i2c9555pin_write(device_mtr, EXT_IO2, 0); // * MTR_FR_IN1
    i2c9555pin_write(device_mtr, EXT_IO3, 1); // * MTR_FR_IN2
    i2c9555pin_write(device_mtr, EXT_IO4, 1); // * MTR_BL_IN1
    i2c9555pin_write(device_mtr, EXT_IO5, 0); // * MTR_BL_IN2
    i2c9555pin_write(device_mtr, EXT_IO6, 0); // * MTR_BR_IN1
    i2c9555pin_write(device_mtr, EXT_IO7, 1); // * MTR_BR_IN2
    // Set motor speeds
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0, MTR_TURN_SPD * mag);   // * Front-Left Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1, MTR_TURN_SPD * mag);   // * Front-Right Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2, MTR_TURN_SPD * mag);   // * Back-Left Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_3, MTR_TURN_SPD * mag);   // * Back-Right Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_3);
}

static void slightTurnLeft()
{
    // Set motor directions for slight left turn
    i2c9555pin_write(device_mtr, EXT_IO0, 1); // * MTR_FL_IN1
    i2c9555pin_write(device_mtr, EXT_IO1, 0); // * MTR_FL_IN2
    i2c9555pin_write(device_mtr, EXT_IO2, 1); // * MTR_FR_IN1
    i2c9555pin_write(device_mtr, EXT_IO3, 0); // * MTR_FR_IN2
    i2c9555pin_write(device_mtr, EXT_IO4, 1); // * MTR_BL_IN1
    i2c9555pin_write(device_mtr, EXT_IO5, 0); // * MTR_BL_IN2
    i2c9555pin_write(device_mtr, EXT_IO6, 1); // * MTR_BR_IN1
    i2c9555pin_write(device_mtr, EXT_IO7, 0); // * MTR_BR_IN2
    // Set motor speeds
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0, MTR_SLOW_SPD);   // * Front-Left Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1, MTR_FULL_SPD);   // * Front-Right Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2, MTR_SLOW_SPD);   // * Back-Left Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_3, MTR_FULL_SPD);   // * Back-Right Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_3);
}

static void slightTurnRight()
{
    // Set motor directions for slight right turn
    i2c9555pin_write(device_mtr, EXT_IO0, 1); // * MTR_FL_IN1
    i2c9555pin_write(device_mtr, EXT_IO1, 0); // * MTR_FL_IN2
    i2c9555pin_write(device_mtr, EXT_IO2, 1); // * MTR_FR_IN1
    i2c9555pin_write(device_mtr, EXT_IO3, 0); // * MTR_FR_IN2
    i2c9555pin_write(device_mtr, EXT_IO4, 1); // * MTR_BL_IN1
    i2c9555pin_write(device_mtr, EXT_IO5, 0); // * MTR_BL_IN2
    i2c9555pin_write(device_mtr, EXT_IO6, 1); // * MTR_BR_IN1
    i2c9555pin_write(device_mtr, EXT_IO7, 0); // * MTR_BR_IN2
    // Set motor speeds
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0, MTR_FULL_SPD);   // * Front-Left Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1, MTR_SLOW_SPD);   // * Front-Right Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2, MTR_FULL_SPD);   // * Back-Left Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_3, MTR_SLOW_SPD);   // * Back-Right Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_3);
}

static void stopMotors()
{
    // Set motor speeds to zero
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0, MTR_STOP);   // * Front-Left Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1, MTR_STOP);   // * Front-Right Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2, MTR_STOP);   // * Back-Left Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_3, MTR_STOP);   // * Back-Right Motor
    ledc_update_duty(LEDC_SPEED_MODE_MAX, LEDC_CHANNEL_3);
}
// ? Motor Movement Control END

// ! Motor Control based on IR State
void mtr_ctrl(void *args)
{
    IRState state;
    while (1)
    {
        state = current_ir_state;
        switch (state)
        {
        case NONE:
            // Stop all motors
            stopMotors();
            break;
        case BOTH_FORWARD:
            // Move forward
            moveForwardFast();
            break;
        case ONLY_L_FORWARD:
            // Adjust to the left
            sharpTurnLeft(0.85);
            break;
        case ONLY_R_FORWARD:
            // Adjust to the right
            sharpTurnRight(0.85);
            break;
        case L_MED_DETECTED:
            // Slight left adjustment
            slightTurnLeft();
            break;
        case R_MED_DETECTED:
            // Slight right adjustment
            slightTurnRight();
            break;
        case L_FIRST_DETECTED:
            // Sharp left turn
            sharpTurnLeft(1.0);
            break;
        case R_FIRST_DETECTED:
            // Sharp right turn
            sharpTurnRight(1.0);
            break;
        case L_SECOND_DETECTED:
            // Very sharp left turn
            sharpTurnLeft(1.35);
            break;
        case R_SECOND_DETECTED:
            // Very sharp right turn
            sharpTurnRight(1.35);
            break;
        case FULL_DETECTED:
            // Start Point & Stop Point Sign, deciding whether start or stop
            if (bar_detected_flag == 0)
            {
                moveForwardFast();
            }
            else if (bar_detected_flag == 1)
            {
                stopMotors();
            }
            else
            {
                ESP_LOGI(TAG, "Bar Detected Flag too large index!");
            }
            
            break;
        default:
            // Default action
            ESP_LOGI(TAG, "Unknown IR State");
            break;
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    
}