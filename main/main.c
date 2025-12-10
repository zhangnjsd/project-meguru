#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "includes/i2c9555.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"

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

// ? IR Init
static esp_err_t i2c_bus_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);

// ? IR Sensor Read
static esp_err_t read_sensor_registers(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *buffer, size_t length);

// ! Motor Control based on IR State
void mtr_ctrl(void *args);

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

    /*
    // * Initialize IR_Module via 9555
    device_ir = i2c9555_add_device(SDA_PIN, SCL_PIN, 0x20, GPIO_NUM_39, ir_read_callback);
    */
    /*  IR Module Pin Configuration
        Pin     Functions       State
        00      IR_LL           Input
        01      IR_LF           Input
        02      IR_FL           Input
        03      IR_FR           Input
        04      IR_RF           Input
        05      IR_RR           Input
    */
    /*
    i2c9555_ioconfig(device_ir, 0xFFFF);
    */

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
    xTaskCreate(mtr_ctrl, "Motor Control Task", 4096, NULL, 3, NULL);
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
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MTR_FULL_SPD); // * Front-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, MTR_FULL_SPD); // * Front-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, MTR_FULL_SPD); // * Back-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, MTR_FULL_SPD); // * Back-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
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
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MTR_TURN_SPD * mag); // * Front-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, MTR_TURN_SPD * mag); // * Front-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, MTR_TURN_SPD * mag); // * Back-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, MTR_TURN_SPD * mag); // * Back-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
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
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MTR_TURN_SPD * mag); // * Front-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, MTR_TURN_SPD * mag); // * Front-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, MTR_TURN_SPD * mag); // * Back-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, MTR_TURN_SPD * mag); // * Back-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
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
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MTR_SLOW_SPD); // * Front-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, MTR_FULL_SPD); // * Front-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, MTR_SLOW_SPD); // * Back-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, MTR_FULL_SPD); // * Back-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
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
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, MTR_FULL_SPD); // * Front-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, MTR_SLOW_SPD); // * Front-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, MTR_FULL_SPD); // * Back-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, MTR_SLOW_SPD); // * Back-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
}

static void stopMotors()
{
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
// ? Motor Movement Control END

// ! Motor Control based on IR State
void mtr_ctrl(void *args)
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
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

// ! Tablet Data Callback Handler
void tablet_data_handler(TabletData data)
{
    ESP_LOGI(TAG, "X: %u, Y: %u, Lifting: %u, mclawSw: %u",
             data.x_value, data.y_value, data.lifting_arm_value, data.mclaw_switch);

    // * Only process tablet data in manual mode
    if (current_mode != CONTROL_MODE_MANUAL)
    {
        ESP_LOGW(TAG, "Tablet data ignored - not in manual mode (current: %d)", current_mode);
        return;
    }

    // ? Manual Control Mode Movement START
    // * Map tablet data to motor controls
    // * X: 0x0000(Backward) <- 0x7F(Center) -> 0xFF(Forward)
    // * Y: 0x0000(Left) <- 0x7F(Center) -> 0xFF(Right)
    uint16_t x = data.x_value;
    uint16_t y = data.y_value;

    // * Check if joystick is at neutral position
    if (x == 0x7F && y == 0x7F)
    {
        stopMotors();
        return;
    }

    // * Determine motor speeds based on x and y values
    uint16_t fl_speed, fr_speed, bl_speed, br_speed;

    // * Motor direction control
    uint8_t fl_in1, fl_in2, fr_in1, fr_in2, bl_in1, bl_in2, br_in1, br_in2;

    // * Case 1: Pure forward (x > 0x7F, y == 0x7F)
    if (x > 0x7F && y == 0x7F)
    {
        moveForwardFast();
        return;
    }

    // * Case 2: Pure backward (x < 0x7F, y == 0x7F)
    if (x < 0x7F && y == 0x7F)
    {
        // Backward
        fl_in1 = 0;
        fl_in2 = 1;
        fr_in1 = 0;
        fr_in2 = 1;
        bl_in1 = 0;
        bl_in2 = 1;
        br_in1 = 0;
        br_in2 = 1;
        fl_speed = fr_speed = bl_speed = br_speed = MTR_FULL_SPD;
    }
    // * Case 3: Pure left (x == 0x7F, y < 0x7F)
    else if (x == 0x7F && y < 0x7F)
    {
        sharpTurnLeft(1.0);
        return;
    }
    // * Case 4: Pure right (x == 0x7F, y > 0x7F)
    else if (x == 0x7F && y > 0x7F)
    {
        sharpTurnRight(1.0);
        return;
    }
    // * Case 5: Forward + Left diagonal
    else if (x > 0x7F && y < 0x7F)
    {
        fl_in1 = 1;
        fl_in2 = 0;
        fr_in1 = 1;
        fr_in2 = 0;
        bl_in1 = 1;
        bl_in2 = 0;
        br_in1 = 1;
        br_in2 = 0;

        // Left side motors slower, right side faster
        fl_speed = (uint16_t)(MTR_FULL_SPD * (0x7F - y) / 127);
        bl_speed = (uint16_t)(MTR_FULL_SPD * (0x7F - y) / 127);
        fr_speed = MTR_FULL_SPD;
        br_speed = MTR_FULL_SPD;
    }
    // * Case 6: Forward + Right diagonal
    else if (x > 0x7F && y > 0x7F)
    {
        fl_in1 = 1;
        fl_in2 = 0;
        fr_in1 = 1;
        fr_in2 = 0;
        bl_in1 = 1;
        bl_in2 = 0;
        br_in1 = 1;
        br_in2 = 0;

        // Right side motors slower, left side faster
        fr_speed = (uint16_t)(MTR_FULL_SPD * (y - 0x7F) / 127);
        br_speed = (uint16_t)(MTR_FULL_SPD * (y - 0x7F) / 127);
        fl_speed = MTR_FULL_SPD;
        bl_speed = MTR_FULL_SPD;
    }
    // * Case 7: Backward + Left diagonal
    else if (x < 0x7F && y < 0x7F)
    {
        fl_in1 = 0;
        fl_in2 = 1;
        fr_in1 = 0;
        fr_in2 = 1;
        bl_in1 = 0;
        bl_in2 = 1;
        br_in1 = 0;
        br_in2 = 1;

        // Left side motors slower, right side faster
        fl_speed = (uint16_t)(MTR_FULL_SPD * (0x7F - y) / 127);
        bl_speed = (uint16_t)(MTR_FULL_SPD * (0x7F - y) / 127);
        fr_speed = MTR_FULL_SPD;
        br_speed = MTR_FULL_SPD;
    }
    // * Case 8: Backward + Right diagonal
    else if (x < 0x7F && y > 0x7F)
    {
        fl_in1 = 0;
        fl_in2 = 1;
        fr_in1 = 0;
        fr_in2 = 1;
        bl_in1 = 0;
        bl_in2 = 1;
        br_in1 = 0;
        br_in2 = 1;

        // Right side motors slower, left side faster
        fr_speed = (uint16_t)(MTR_FULL_SPD * (y - 0x7F) / 127);
        br_speed = (uint16_t)(MTR_FULL_SPD * (y - 0x7F) / 127);
        fl_speed = MTR_FULL_SPD;
        bl_speed = MTR_FULL_SPD;
    }
    else
    {
        stopMotors();
        return;
    }

    // * Set motor directions
    i2c9555pin_write(device_mtr, EXT_IO0, fl_in1); // * MTR_FL_IN1
    i2c9555pin_write(device_mtr, EXT_IO1, fl_in2); // * MTR_FL_IN2
    i2c9555pin_write(device_mtr, EXT_IO2, fr_in1); // * MTR_FR_IN1
    i2c9555pin_write(device_mtr, EXT_IO3, fr_in2); // * MTR_FR_IN2
    i2c9555pin_write(device_mtr, EXT_IO4, bl_in1); // * MTR_BL_IN1
    i2c9555pin_write(device_mtr, EXT_IO5, bl_in2); // * MTR_BL_IN2
    i2c9555pin_write(device_mtr, EXT_IO6, br_in1); // * MTR_BR_IN1
    i2c9555pin_write(device_mtr, EXT_IO7, br_in2); // * MTR_BR_IN2

    // * Set motor speeds
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, fl_speed); // * Front-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1, fr_speed); // * Front-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_1);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, bl_speed); // * Back-Left Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, br_speed); // * Back-Right Motor
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);
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