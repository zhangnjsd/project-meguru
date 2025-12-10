#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "includes/i2c9555.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include <math.h>

#include "includes/nimble.h"
#include "includes/gap_config.h"
#include "includes/gatt_service.h"

#ifndef TAG
#define TAG "MAIN"
#endif

#define SDA_PIN GPIO_NUM_8 /*!< SDA BUS*/
#define SCL_PIN GPIO_NUM_9 /*!< SDA BUS*/
#define I2C_FREQ_HZ 100000 /*!< I2C Frequency 100kHz*/
#define I2C_TIMEOUT_MS 1000 /*!< I2C Timeout in milliseconds*/

#define I2C_9555_ADDRESS 0x20 /* I2C9555 Address */

#define MTR_FL_PWM GPIO_NUM_10 /*!< Front-Left Motor*/
#define MTR_FR_PWM GPIO_NUM_11 /*!< Front-Right Motor*/
#define MTR_BL_PWM GPIO_NUM_12 /*!< Back-Left Motor*/
#define MTR_BR_PWM GPIO_NUM_13 /*!< Back-Right Motor*/

#define ARM_LIFT_PWM GPIO_NUM_4 /*!< Lifting Arm Motor*/
#define CLAW_SW_PWM GPIO_NUM_5  /*!< Mechanical Claw Motor*/

#define SERVO_FREQ_HZ 50                          /*!< Servo Frequency 50Hz */
#define SERVO_PERIOD_US (1000000 / SERVO_FREQ_HZ) /*!< Servo Period in microseconds */
#define SERVO_MIN_PULSE_US 500                    /*!< Servo Minimum Pulse Width in microseconds */
#define SERVO_MAX_PULSE_US 2500                   /*!< Servo Maximum Pulse Width in microseconds */
#define SERVO_FULL_RANGE_DEG 270                  /*!< Servo Full Range in degrees */
#define CLAW_OPEN_DEG 75                          /*!< Claw Open Position in degrees */

#define MTR_FULL_SPD 4096 /*!< Full Speed Duty Cycle */
#define MTR_TURN_SPD 2048 /*!< Turn Speed Duty Cycle */
#define MTR_SLOW_SPD 1024 /*!< Slow Speed Duty Cycle */
#define MTR_STOP 0        /*!< Stop Duty Cycle */

#define SENSOR_I2C_ADDRESS 0x5D                      /* IR I2C Address */
#define SENSOR_CHANNEL_COUNT 8                       /* Number of IR Channels */
#define SENSOR_STATE_REG 5u                          /* Sensor State Register */
#define SENSOR_ANALOG_REG 6u                         /* Sensor Analog Data Register */
#define SENSOR_TRESHOLD_REG 22u                      /* Sensor Threshold Register */
#define SENSOR_DATA_BYTES (SENSOR_CHANNEL_COUNT * 2) /* Number of Data Bytes */

// int device_ir;
int device_mtr;

int bar_detected_flag = 0;

// IR _xTask
TaskHandle_t ir_task_handle = NULL;

/* ControlMode and current_mode are now defined in gatt_service.h */
volatile ControlMode current_mode = CONTROL_MODE_AUTO;
volatile bool manual_control_active = false;

// ? Line-following State Machine
typedef enum
{
    NONE,              /*!<Priority: 0 No IR Sensor Detected Line*/
    BOTH_FORWARD,      /*!<Priority: 2 Center Sensors (FL & FR) Detected - Straight Line*/
    ONLY_L_FORWARD,    /*!<Priority: 2 Only Left Center Sensor (FL) Detected*/
    ONLY_R_FORWARD,    /*!<Priority: 2 Only Right Center Sensor (FR) Detected*/
    L_MED_DETECTED,    /*!<Priority: 3 Left Medium Layer (LF & FR) Detected - Early Correction*/
    R_MED_DETECTED,    /*!<Priority: 3 Right Medium Layer (RF & FR) Detected - Early Correction*/
    L_FIRST_DETECTED,  /*!<Priority: 3 Only Left First Layer (LF) Detected*/
    R_FIRST_DETECTED,  /*!<Priority: 3 Only Right First Layer (RF) Detected*/
    L_SECOND_DETECTED, /*!<Priority: 4 Left Second Layer (LL) Detected - Sharp Turn*/
    R_SECOND_DETECTED, /*!<Priority: 4 Right Second Layer (RR) Detected - Sharp Turn*/
    L_FIRST_SECOND_DETECTED, /*!<Priority: 4 Left Multiple Layers (LL & LF) Detected*/
    R_FIRST_SECOND_DETECTED, /*!<Priority: 4 Right Multiple Layers (RR & RF) Detected*/
    L_3RD_DETECTED,   /*!<Priority: 5 Left Outermost Layer (LLL) Detected - Extreme Turn*/
    R_3RD_DETECTED,   /*!<Priority: 5 Right Outermost Layer (RRR) Detected - Extreme Turn*/
    L_SECOND_3RD_DETECTED, /*!<Priority: 5 Left Extreme (LL & LLL) Detected - Maximum Turn*/
    R_SECOND_3RD_DETECTED, /*!<Priority: 5 Right Extreme (RR & RRR) Detected - Maximum Turn*/
    FULL_DETECTED,     /*!<Priority: 1 (Highest) All Outermost Sensors (LLL & RRR) Detected - Line Start/End*/
} IRState;

// ? IR Data
typedef struct
{
    bool LLL;
    bool LL;
    bool LF;
    bool FL;
    bool FR;
    bool RF;
    bool RR;
    bool RRR;
} IRData;

// ? Motor Data
/*
!!! Mecanum Wheel Basic Motion Chart
!----------------------------------------------------------------------------------------------------------------------------------------------------------------!
? Motion Pattern            |   Wheel Rotation Relationship                                                                         | Arrow Direction in Diagram
!----------------------------------------------------------------------------------------------------------------------------------------------------------------!
* Forward                   |   Four wheels rotate forward at the same speed                                                        | →
* Backward                  |   Four wheels rotate in opposite directions at the same speed                                         | ←
* Right Translation         |   Left front/right rear wheel rotates forward, right front/left rear wheel rotates counter-clockwise  | →←
* Left Translation          |   Right front/left rear wheel rotates forward, left front/right rear wheel rotates counter-clockwise  | ←→
* Spot Rotation (Forward)   |   Left wheel rotates counter-clockwise, right wheel rotates forward                                   | ↻
* Spot Reverse Rotation     |   Right wheel rotates counter-clockwise, left wheel rotates forward                                   | ↺
!----------------------------------------------------------------------------------------------------------------------------------------------------------------!
*/
// * Motor Property
typedef struct 
{
    ledc_channel_t pwm_channel;
    uint16_t speed;
    gpio_num_t in1_pin;
    gpio_num_t in2_pin;
    uint8_t in1_level;
    uint8_t in2_level;
} Motor;
// * General Structure
typedef struct
{
    Motor FrontL;
    Motor FrontR;
    Motor BackL;
    Motor BackR;
    float vx; // Velocity in X direction (cm/s)
    float vy; // Velocity in Y direction (cm/s)
    float vr; // Rotational Velocity (rad/s)
} MotorGroup;

volatile IRData current_ir_data;
volatile IRState current_ir_state = NONE;

// Shared I2C resources (initialized once and reused by IR + 9555)
static i2c_master_bus_handle_t shared_i2c_bus = NULL;
static i2c_master_dev_handle_t ir_dev_handle = NULL;

static uint16_t servo_duty_from_pulse_us(uint32_t pulse_us)
{
    uint32_t duty = (pulse_us * MTR_FULL_SPD) / SERVO_PERIOD_US;
    if (duty > MTR_FULL_SPD)
    {
        duty = MTR_FULL_SPD;
    }
    return (uint16_t)duty;
}

// ! Tablet Data Handler
void tablet_data_handler(TabletData data);

// ! IR Module Input Callback
void ir_read_callback(uint8_t state_buf[0]);

// ! IR Module Read Task
void ir_read_task();

// ? Mecanum Wheel Movement Function
static void mecanum_move(MotorGroup* motor, float mag);

// ? IR Init
static esp_err_t i2c_bus_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);

// ? IR Sensor Read
static esp_err_t read_sensor_registers(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *buffer, size_t length);

// ! Motor Control based on IR State
void mtr_ctrl_ir(void *args);

void app_main(void)
{

    current_ir_data = (IRData){
        .LLL = false,
        .LL = false,
        .LF = false,
        .FL = false,
        .FR = false,
        .RF = false,
        .RR = false,
        .RRR = false,
    };

    // Initialize I2C bus once (shared by IR sensor + 9555 expander)
    ESP_ERROR_CHECK(i2c_bus_init(&shared_i2c_bus, &ir_dev_handle));
    ESP_ERROR_CHECK(i2c9555_attach_bus(shared_i2c_bus));

    // * Initialize Motor Controller via 9555
    device_mtr = i2c9555_add_device(SDA_PIN, SCL_PIN, I2C_9555_ADDRESS, GPIO_NUM_NC, NULL);
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
    ESP_LOGI(TAG, "Configuring Motor Controller IO Expander");
    i2c9555_ioconfig(device_mtr, 0x0000);

    // * PWM Initialization
    ESP_LOGI(TAG, "Configuring LEDC PWM Timers and Channels");
    ledc_timer_config_t pwm_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_0,
        .duty_resolution = LEDC_TIMER_12_BIT, // 4096 levels
        .freq_hz = 5000,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&pwm_timer);

    // * Dedicated servo timer for GPIO4 (Lifting Arm)
    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&servo_timer);

    // * PWM Channel Configuration
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
    ESP_LOGI(TAG, "Configuring Motor PWM Channels");
    ledc_channel_config(&mtr_fl_channel);
    ledc_channel_config(&mtr_fr_channel);
    ledc_channel_config(&mtr_bl_channel);
    ledc_channel_config(&mtr_br_channel);

    // * PWM Channel Configuration for Lifting Arm
    ESP_LOGI(TAG, "Configuring Lifting Arm PWM Channel");
    ledc_channel_config_t arm_lift_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_4,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = ARM_LIFT_PWM,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&arm_lift_channel);

    // * PWM Channel Configuration for Mechanical Claw
    ledc_channel_config_t claw_sw_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_5,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = CLAW_SW_PWM,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&claw_sw_channel);

    // ? BLE Handler START
    ESP_LOGI(TAG, "Initializing NimBLE Stack");
    // * NimBLE initialization
    nimble_init();

    // * Bluetooth Callback Registration
    gatt_service_register_callback(tablet_data_handler);
    // ? BLE Handler END

    // * IR Read Thread
    ESP_LOGI(TAG, "Starting IR Read Task");
    // Increase stack to avoid overflow from logging and I2C routines
    xTaskCreate(ir_read_task, "IR Read Task", 4096, NULL, 2, &ir_task_handle);

    // * Motor Control Thread
    ESP_LOGI(TAG, "Starting Motor Control Task");
    // Increase stack to accommodate motor control logic and logging
    xTaskCreate(mtr_ctrl_ir, "Motor Control Task", 4096, NULL, 3, NULL);
}

// ? Define Motor Group
volatile MotorGroup mecanum = {
    .FrontL = {
        .in1_pin = EXT_IO0,
        .in2_pin = EXT_IO1,
        .pwm_channel = LEDC_CHANNEL_0,
        .speed = 0,
        .in1_level = 0,
        .in2_level = 0,
    },
    .FrontR = {
        .in1_pin = EXT_IO2,
        .in2_pin = EXT_IO3,
        .pwm_channel = LEDC_CHANNEL_1,
        .speed = 0,
        .in1_level = 0,
        .in2_level = 0,
    },
    .BackL = {
        .in1_pin = EXT_IO4,
        .in2_pin = EXT_IO5,
        .pwm_channel = LEDC_CHANNEL_2,
        .speed = 0,
        .in1_level = 0,
        .in2_level = 0,
    },
    .BackR = {
        .in1_pin = EXT_IO6,
        .in2_pin = EXT_IO7,
        .pwm_channel = LEDC_CHANNEL_3,
        .speed = 0,
        .in1_level = 0,
        .in2_level = 0,
    },
    .vx = 0.0f,
    .vy = 0.0f,
    .vr = 0.0f,
};

// ? Motor Controller
esp_err_t mtr_spd_setting(MotorGroup* motor) {
    // ERROR?
    esp_err_t err;

    // * FL
    err = i2c9555pin_write(device_mtr, motor->FrontL.in1_pin, motor->FrontL.in1_level);
    err = i2c9555pin_write(device_mtr, motor->FrontL.in2_pin, motor->FrontL.in2_level);
    err = ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->FrontL.pwm_channel, motor->FrontL.speed);
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->FrontL.pwm_channel);

    // * FR
    err = i2c9555pin_write(device_mtr, motor->FrontR.in1_pin, motor->FrontR.in1_level);
    err = i2c9555pin_write(device_mtr, motor->FrontR.in2_pin, motor->FrontR.in2_level);
    err = ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->FrontR.pwm_channel, motor->FrontR.speed);
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->FrontR.pwm_channel);

    // * BL
    err = i2c9555pin_write(device_mtr, motor->BackL.in1_pin, motor->BackL.in1_level);
    err = i2c9555pin_write(device_mtr, motor->BackL.in2_pin, motor->BackL.in2_level);
    err = ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->BackL.pwm_channel, motor->BackL.speed);
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->BackL.pwm_channel);

    // * BR
    err = i2c9555pin_write(device_mtr, motor->BackR.in1_pin, motor->BackR.in1_level);
    err = i2c9555pin_write(device_mtr, motor->BackR.in2_pin, motor->BackR.in2_level);
    err = ledc_set_duty(LEDC_LOW_SPEED_MODE, motor->BackR.pwm_channel, motor->BackR.speed);
    err = ledc_update_duty(LEDC_LOW_SPEED_MODE, motor->BackR.pwm_channel);

    if (err)
    {
        return err;
    } else {
        return ESP_OK;
    }

}

// ! IR Module Read Task
void ir_read_task()
{
    ESP_LOGI(TAG, "IR Read Task Started");
    if (ir_dev_handle == NULL)
    {
        ESP_LOGE(TAG, "IR device handle is NULL; ensure i2c_bus_init runs before tasks start");
    }

    esp_err_t err;
    uint8_t state_buf[1] = {0};

    while (1)
    {
        vTaskDelay(pdMS_TO_TICKS(10));
        if (ir_dev_handle == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(20));
            continue;
        }

        err = read_sensor_registers(ir_dev_handle, SENSOR_STATE_REG, state_buf, sizeof(state_buf));
        if (err == ESP_OK) {
            ir_read_callback(state_buf);
        } else {
            ESP_LOGE(TAG, "Failed to read state register, error code: 0x%X", err);
        }
        vTaskDelay(pdMS_TO_TICKS(20));
    }
}

// ? Mecanum Wheel Movement Function
/**
 * @brief Mecanum wheel omnidirectional movement with rotation
 * @param motor Pointer to MotorGroup structure (reads vx, vy, vr from struct)
 * @param mag Speed magnitude multiplier (0.0 ~ 1.0)
 * 
 * Before calling, set motor->vx, motor->vy, motor->vr:
 *   vx: Lateral velocity (-1.0 ~ 1.0, positive = right)
 *   vy: Longitudinal velocity (-1.0 ~ 1.0, positive = forward)
 *   vr: Angular velocity (-1.0 ~ 1.0, positive = clockwise)
 * 
 * ! Mecanum wheel kinematics:
 * ? FL = Vy + Vx + ω
 * ? FR = Vy - Vx - ω
 * ? BL = Vy - Vx + ω
 * ? BR = Vy + Vx - ω
 */
static void mecanum_move(MotorGroup* motor, float mag)
{
    // Read velocity values from MotorGroup structure
    float vx = motor->vx;
    float vy = motor->vy;
    float omega = motor->vr;
    
    // Calculate each wheel speed (-1.0 ~ 1.0)
    float fl = vy + vx + omega;
    float fr = vy - vx - omega;
    float bl = vy - vx + omega;
    float br = vy + vx - omega;
    
    // Normalize to ensure max value does not exceed 1.0
    float max_val = fmaxf(fmaxf(fabsf(fl), fabsf(fr)), fmaxf(fabsf(bl), fabsf(br)));
    if (max_val > 1.0f)
    {
        fl /= max_val;
        fr /= max_val;
        bl /= max_val;
        br /= max_val;
    }
    
    // Apply speed magnitude
    fl *= mag;
    fr *= mag;
    bl *= mag;
    br *= mag;
    
    // Set front-left wheel
    motor->FrontL.in1_level = (fl >= 0) ? 1 : 0;
    motor->FrontL.in2_level = (fl >= 0) ? 0 : 1;
    motor->FrontL.speed = (uint16_t)(fabsf(fl) * MTR_FULL_SPD);
    
    // Set front-right wheel
    motor->FrontR.in1_level = (fr >= 0) ? 1 : 0;
    motor->FrontR.in2_level = (fr >= 0) ? 0 : 1;
    motor->FrontR.speed = (uint16_t)(fabsf(fr) * MTR_FULL_SPD);
    
    // Set back-left wheel
    motor->BackL.in1_level = (bl >= 0) ? 1 : 0;
    motor->BackL.in2_level = (bl >= 0) ? 0 : 1;
    motor->BackL.speed = (uint16_t)(fabsf(bl) * MTR_FULL_SPD);
    
    // Set back-right wheel
    motor->BackR.in1_level = (br >= 0) ? 1 : 0;
    motor->BackR.in2_level = (br >= 0) ? 0 : 1;
    motor->BackR.speed = (uint16_t)(fabsf(br) * MTR_FULL_SPD);
    
    mtr_spd_setting(motor);
}

// ! IR Module Input Callback
void ir_read_callback(uint8_t state_buf[0])
{
    /*
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
    */

    // * Update IR Data
    for (size_t i = 0; i < SENSOR_CHANNEL_COUNT; ++i) {
        bool state = (state_buf[0] >> i) & 0x01;
        switch (i) {
            case 0:
                current_ir_data.LLL = state;
                break;
            case 1:
                current_ir_data.LL = state;
                break;
            case 2:
                current_ir_data.LF = state;
                break;
            case 3:
                current_ir_data.FL = state;
                break;
            case 4:
                current_ir_data.FR = state;
                break;
            case 5:
                current_ir_data.RF = state;
                break;
            case 6:
                current_ir_data.RR = state;
                break;
            case 7:
                current_ir_data.RRR = state;
                break;
            default:
                break;
        }
    }

    // * Update IR State Machine - FULL_DETECTED has absolute highest priority
    // * All sensors detecting = line end/corner detection
    if (current_ir_data.RRR && current_ir_data.LLL)
    {
        current_ir_state = FULL_DETECTED;
    }
    // * Center sensors (FL, FR) have second highest priority for straight-line stability
    else if (current_ir_data.FL && current_ir_data.FR)
    {
        current_ir_state = BOTH_FORWARD;
    }
    else if (current_ir_data.FL)
    {
        current_ir_state = ONLY_L_FORWARD;
    }
    else if (current_ir_data.FR)
    {
        current_ir_state = ONLY_R_FORWARD;
    }
    // * Medium layer (LF, RF) for early correction
    else if (current_ir_data.LF && current_ir_data.RF)
    {
        current_ir_state = BOTH_FORWARD;
    }
    else if (current_ir_data.LF && current_ir_data.FR)
    {
        current_ir_state = L_MED_DETECTED;
    }
    else if (current_ir_data.RF && current_ir_data.FR)
    {
        current_ir_state = R_MED_DETECTED;
    }
    else if (current_ir_data.LF)
    {
        current_ir_state = L_FIRST_DETECTED;
    }
    else if (current_ir_data.RF)
    {
        current_ir_state = R_FIRST_DETECTED;
    }
    // * Outer layer (LL, RR) for sharp turns
    else if (current_ir_data.LL && current_ir_data.LF)
    {
        current_ir_state = L_FIRST_SECOND_DETECTED;
    }
    else if (current_ir_data.RR && current_ir_data.RF)
    {
        current_ir_state = R_FIRST_SECOND_DETECTED;
    }
    else if (current_ir_data.LL)
    {
        current_ir_state = L_SECOND_DETECTED;
    }
    else if (current_ir_data.RR)
    {
        current_ir_state = R_SECOND_DETECTED;
    }
    // * Outermost layer (LLL, RRR) for extreme turns or corner detection
    else if (current_ir_data.LLL && current_ir_data.LL)
    {
        current_ir_state = L_SECOND_3RD_DETECTED;
    }
    else if (current_ir_data.RRR && current_ir_data.RR)
    {
        current_ir_state = R_SECOND_3RD_DETECTED;
    }
    else if (current_ir_data.LLL)
    {
        current_ir_state = L_3RD_DETECTED;
    }
    else if (current_ir_data.RRR)
    {
        current_ir_state = R_3RD_DETECTED;
    }
    else
    {
        current_ir_state = NONE;
    }
}

// ? IR Init
static esp_err_t i2c_bus_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    if (bus_handle == NULL || dev_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (*bus_handle == NULL)
    {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = SDA_PIN,
            .scl_io_num = SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));
        ESP_LOGI(TAG, "I2C master bus initialized once (port %d)", bus_config.i2c_port);
    }
    else
    {
        ESP_LOGI(TAG, "Reusing existing I2C master bus handle");
    }

    if (*dev_handle == NULL)
    {
        i2c_device_config_t dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = SENSOR_I2C_ADDRESS,
            .scl_speed_hz = I2C_FREQ_HZ,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
        ESP_LOGI(TAG, "IR device added to shared I2C bus (addr 0x%02X)", SENSOR_I2C_ADDRESS);
    }
    else
    {
        ESP_LOGI(TAG, "IR device handle already initialized; skipping re-add");
    }

    return ESP_OK;
}

// ? IR Sensor Read
static esp_err_t read_sensor_registers(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *buffer, size_t length)
{
	uint8_t addr[1] = {reg_addr};
	esp_err_t err = i2c_master_transmit_receive(dev_handle, addr, 1, buffer, length, I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
	if (err != ESP_OK) {
		return err;
	}
	return ESP_OK;
}

// ? Motor Movement Control START
static void moveForwardFast()
{
    // vx=0, vy=1 (forward), vr=0
    mecanum.vx = 0.0f;
    mecanum.vy = 1.0f;
    mecanum.vr = 0.0f;
    mecanum_move((MotorGroup*)&mecanum, 1.0f);
}

static void sharpTurnLeft(float mag)
{
    // vx=0, vy=0, vr=-1 (counter-clockwise)
    mecanum.vx = 0.0f;
    mecanum.vy = 0.0f;
    mecanum.vr = -1.0f;
    mecanum_move((MotorGroup*)&mecanum, mag * 0.5f);
}

static void sharpTurnRight(float mag)
{
    // vx=0, vy=0, vr=1 (clockwise)
    mecanum.vx = 0.0f;
    mecanum.vy = 0.0f;
    mecanum.vr = 1.0f;
    mecanum_move((MotorGroup*)&mecanum, mag * 0.5f);
}

static void slightTurnLeft()
{
    // vx=0, vy=1 (forward), vr=-0.3 (slight counter-clockwise)
    mecanum.vx = 0.0f;
    mecanum.vy = 1.0f;
    mecanum.vr = -0.3f;
    mecanum_move((MotorGroup*)&mecanum, 0.8f);
}

static void slightTurnRight()
{
    // vx=0, vy=1 (forward), vr=0.3 (slight clockwise)
    mecanum.vx = 0.0f;
    mecanum.vy = 1.0f;
    mecanum.vr = 0.3f;
    mecanum_move((MotorGroup*)&mecanum, 0.8f);
}

static void shift_left(float mag)
{
    // vx=-1 (left), vy=0, vr=0
    mecanum.vx = -1.0f;
    mecanum.vy = 0.0f;
    mecanum.vr = 0.0f;
    mecanum_move((MotorGroup*)&mecanum, mag * 0.5f);
}

static void shift_right(float mag)
{
    // vx=1 (right), vy=0, vr=0
    mecanum.vx = 1.0f;
    mecanum.vy = 0.0f;
    mecanum.vr = 0.0f;
    mecanum_move((MotorGroup*)&mecanum, mag * 0.5f);
}

static void rotate_clockwise(float mag)
{
    // vx=0, vy=0, vr=1 (clockwise)
    mecanum.vx = 0.0f;
    mecanum.vy = 0.0f;
    mecanum.vr = 1.0f;
    mecanum_move((MotorGroup*)&mecanum, mag * 0.5f);
}

static void rotate_counterclockwise(float mag)
{
    // vx=0, vy=0, vr=-1 (counter-clockwise)
    mecanum.vx = 0.0f;
    mecanum.vy = 0.0f;
    mecanum.vr = -1.0f;
    mecanum_move((MotorGroup*)&mecanum, mag * 0.5f);
}

static void stopMotors()
{
    // All velocities zero = stop
    mecanum.vx = 0.0f;
    mecanum.vy = 0.0f;
    mecanum.vr = 0.0f;
    mecanum_move((MotorGroup*)&mecanum, 0.0f);
}
// ? Motor Movement Control END

// ! Motor Control based on IR State
void mtr_ctrl_ir(void *args)
{
    current_mode = CONTROL_MODE_AUTO;
    bool init_flag = false;
    IRState state;
    IRState lastState = NONE;
    while (1)
    {
        if (init_flag)
        {
            lastState = state;
        }
        state = current_ir_state;
        init_flag = true;
        
        // Only update motor control when state changes
        if (state != lastState)
        {
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
            case L_FIRST_SECOND_DETECTED:
                // Strong left correction
                sharpTurnLeft(1.2);
                break;
            case R_FIRST_SECOND_DETECTED:
                // Strong right correction
                sharpTurnRight(1.2);
                break;
            case L_3RD_DETECTED:
                // Extreme left turn
                sharpTurnLeft(1.5);
                break;
            case R_3RD_DETECTED:
                // Extreme right turn
                sharpTurnRight(1.5);
                break;
            case L_SECOND_3RD_DETECTED:
                // Maximum left turn
                sharpTurnLeft(1.7);
                break;
            case R_SECOND_3RD_DETECTED:
                // Maximum right turn
                sharpTurnRight(1.7);
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

                    // * Switch to Manual Control Mode
                    current_mode = CONTROL_MODE_MANUAL;
                    manual_control_active = true;

                    // * Exit the loop
                    vTaskDelete(NULL);
                    vTaskDelete(ir_task_handle);
                    ESP_LOGI(TAG, "Switching to Manual Control Mode, stopping IR and Motor Control Tasks");
                    // Delete IR I2C Addr
                    i2c_master_bus_rm_device(ir_dev_handle);
                    ESP_LOGI(TAG, "IR device removed from I2C bus");
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
        }

        if (init_flag)
        {
            if (lastState == FULL_DETECTED && state != FULL_DETECTED)
            {
                bar_detected_flag++;
            }
        }

        // Small delay to ensure idle task runs and watchdog is fed
        vTaskDelay(pdMS_TO_TICKS(5));
    }
}

// ! Tablet Data Callback Handler
void tablet_data_handler(TabletData data)
{
    ESP_LOGI(TAG, "X: %u, Y: %u, R: %u, Lifting: %u, mclawSw: %u",
             data.x_value, data.y_value, data.r_value, data.lifting_arm_value, data.mclaw_switch);

    // * Only process tablet data in manual mode
    if (current_mode != CONTROL_MODE_MANUAL)
    {
        ESP_LOGW(TAG, "Tablet data ignored - not in manual mode (current: %d)", current_mode);
        return;
    }

    // ? Manual Control Mode Movement START
    // * Map joystick to mecanum wheel translation
    // * X: 0x00(Left) <- 0x7F(Center) -> 0xFF(Right) => vx (lateral)
    // * Y: 0x00(Backward) <- 0x7F(Center) -> 0xFF(Forward) => vy (longitudinal)
    // * R: 0x00(CCW) <- 0x7F(Center) -> 0xFF(CW) => vr (rotational)
    mecanum.vx = ((float)data.x_value - 127.0f) / 127.0f;
    mecanum.vy = ((float)data.y_value - 127.0f) / 127.0f;
    mecanum.vr = ((float)data.r_value - 127.0f) / 127.0f;
    mecanum_move((MotorGroup*)&mecanum, 1.0f);
    // ? Manual Control Mode Movement END

    // ? Lifting Arm Control START
    uint16_t lifting_raw = data.lifting_arm_value;
    const uint16_t lifting_max_input = 225;
    if (lifting_raw > lifting_max_input)
    {
        lifting_raw = lifting_max_input;
    }
    uint32_t pulse_range = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
    uint32_t pulse_us = SERVO_MIN_PULSE_US + ((uint32_t)lifting_raw * pulse_range) / lifting_max_input;
    uint16_t lift_duty = servo_duty_from_pulse_us(pulse_us);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, lift_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);
    // ? Lifting Arm Control END

    // ? Mechanical Claw Control START
    uint8_t claw_switch = data.mclaw_switch;
    uint16_t claw_target_deg = claw_switch ? CLAW_OPEN_DEG : 0;
    if (claw_target_deg > SERVO_FULL_RANGE_DEG)
    {
        claw_target_deg = SERVO_FULL_RANGE_DEG;
    }
    uint32_t claw_pulse_range = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
    uint32_t claw_pulse_us = SERVO_MIN_PULSE_US + ((uint32_t)claw_target_deg * claw_pulse_range) / SERVO_FULL_RANGE_DEG;
    uint16_t claw_duty = servo_duty_from_pulse_us(claw_pulse_us);

    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, claw_duty);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5);
    // ? Mechanical Claw Control END
}