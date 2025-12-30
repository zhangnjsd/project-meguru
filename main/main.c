#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "includes/i2c9555.h"
#include "esp_log.h"
#include "esp_err.h"
#include "driver/ledc.h"
#include "driver/mcpwm_prelude.h"
#include <math.h>

#include "includes/nimble.h"
#include "includes/gap_config.h"
#include "includes/gatt_service.h"

#ifndef TAG
#define TAG "MAIN"
#endif

#define TCA_SDA_PIN GPIO_NUM_8 /*!< Ext SDA Pin*/
#define TCA_SCL_PIN GPIO_NUM_9 /*!< Ext SDA Pin*/
#define I2C_FREQ_HZ 100000 /*!< I2C Frequency 100kHz*/
#define I2C_TIMEOUT_MS 1000 /*!< I2C Timeout in milliseconds*/

#define IR_SCL_PIN GPIO_NUM_6 /*!< IR Sensor SCL Pin*/
#define IR_SDA_PIN GPIO_NUM_7 /*!< IR Sensor SDA Pin*/

// ! Choose I2C address based on 9555 model: 
// ; MODEL = 0 for TCA9555
// ; MODEL = 1 for PCA9555
#define TARGET_9555_MODEL(MODEL) ((MODEL) ? 0x20 : 0x40)

#define I2C_9555_ADDRESS TARGET_9555_MODEL(1) /* I2C9555 Address */

#define MTR_FL_PWM GPIO_NUM_10 /*!< Front-Left Motor*/
#define MTR_FR_PWM GPIO_NUM_11 /*!< Front-Right Motor*/
#define MTR_BL_PWM GPIO_NUM_12 /*!< Back-Left Motor*/
#define MTR_BR_PWM GPIO_NUM_13 /*!< Back-Right Motor*/

#define SERVO_FREQ_HZ 50                          /*!< Servo Frequency 50Hz */
#define SERVO_PERIOD_US (1000000 / SERVO_FREQ_HZ) /*!< Servo Period in microseconds */
#define SERVO_MIN_PULSE_US 500                    /*!< Servo Minimum Pulse Width in microseconds */
#define SERVO_MAX_PULSE_US 2500                   /*!< Servo Maximum Pulse Width in microseconds */
#define SERVO_FULL_RANGE_DEG 270                  /*!< Servo Full Range in degrees */
#define CLAW_OPEN_DEG 75                          /*!< Claw Open Position in degrees */

#define MTR_FULL_SPD 4096 /*!< Full Speed Duty Cycle (12-bit resolution) */
#define MTR_MIN_START 3120 /*!< Minimum PWM to start motor (~18.3V at 24V supply = 76%) */
#define MTR_INPUT_DEADZONE 0.05f /*!< Input deadzone (5%) - below this, motor stops */
#define MTR_TURN_SPD 2048 /*!< Turn Speed Duty Cycle */
#define MTR_SLOW_SPD 1024 /*!< Slow Speed Duty Cycle */
#define MTR_STOP 0        /*!< Stop Duty Cycle */

#define SENSOR_I2C_ADDRESS 0x5D                      /* IR I2C Address */
#define SENSOR_CHANNEL_COUNT 8                       /* Number of IR Channels */
#define SENSOR_STATE_REG 5u                          /* Sensor State Register */
#define SENSOR_ANALOG_REG 6u                         /* Sensor Analog Data Register */
#define SENSOR_TRESHOLD_REG 22u                      /* Sensor Threshold Register */
#define SENSOR_DATA_BYTES (SENSOR_CHANNEL_COUNT * 2) /* Number of Data Bytes */

#define SERVO_LIFT_A_PIN GPIO_NUM_35 /*!< Lifting Arm A Servo Pin */
#define SERVO_LIFT_B_PIN GPIO_NUM_36 /*!< Lifting Arm B Servo Pin */
#define SERVO_LIFT_C_PIN GPIO_NUM_1 /*!< Lifting Arm C Servo Pin */

#define SERVO_LIFT_END_PIN GPIO_NUM_2 /*!< Lifting Arm End Effector Servo Pin */

#define SERVO_CLAW_PIN GPIO_NUM_4 /*!< Mechanical Claw Servo Pin */

// ! Servo Pulse Width Ranges (0x00-0xFF maps to these ranges)
// ! Adjust these values to calibrate the servo positions (range 500us to 2500us)
#define SERVO_A_MIN_US 500 /*!< Minimum pulse width for Lifting Arm A */
#define SERVO_A_MAX_US 1620 /*!< Maximum pulse width for Lifting Arm A */

#define SERVO_B_MIN_US 750 /*!< Minimum pulse width for Lifting Arm B */
#define SERVO_B_MAX_US 2500 /*!< Maximum pulse width for Lifting Arm B */

// todo: Calibrate these values for your specific servos
#define SERVO_C_MIN_US 500 /*!< Minimum pulse width for Lifting Arm C */
#define SERVO_C_MAX_US 2500 /*!< Maximum pulse width for Lifting Arm C */

// todo: Calibrate these values for your specific servos
#define SERVO_END_MIN_US 500 /*!< Minimum pulse width for Lifting Arm C */
#define SERVO_END_MAX_US 2500 /*!< Maximum pulse width for Lifting Arm C */

#define SERVO_CLAW_MIN_US 500 /*!< Minimum pulse width for Mechanical Claw */
#define SERVO_CLAW_MAX_US 950 /*!< Maximum pulse width for Mechanical Claw */

int device_mtr;

// * MCPWM Handles for Motors
mcpwm_cmpr_handle_t mtr_cmpr_fl = NULL;
mcpwm_cmpr_handle_t mtr_cmpr_fr = NULL;
mcpwm_cmpr_handle_t mtr_cmpr_bl = NULL;
mcpwm_cmpr_handle_t mtr_cmpr_br = NULL;

int bar_detected_flag = 0;

// * IR _xTask
TaskHandle_t ir_task_handle = NULL;

volatile ControlMode current_mode = CONTROL_MODE_AUTO;
volatile bool manual_control_active = false;
volatile bool waiting_for_start = true;

QueueHandle_t tablet_queue = NULL;

// ? Line-following using weighted position algorithm
// * Sensor weights for position calculation (from left to right)
// ; LLL=-4, LL=-3, LF=-2, FL=-1, FR=+1, RF=+2, RR=+3, RRR=+4
typedef enum
{
    LINE_NONE,         /*!< No line detected */
    LINE_FOLLOWING,    /*!< Normal line following */
    LINE_FULL,         /*!< Full bar detected (start/stop) */
} LineState;

typedef struct {
    float position;     
    float last_position;
    int sensor_count;   
    bool full_detected; 
} LineFollowData;

volatile LineFollowData line_data = {0};

// PD Controller parameters
#define LINE_KP 0.9f            // * Proportional gain
#define LINE_KD 0.1f            // * Derivative gain
#define LINE_BASE_SPEED 0.5f    // * Base forward speed

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
/**
*!!! Mecanum Wheel Basic Motion Chart
*!----------------------------------------------------------------------------------------------------------------------------------------------------------------!
*? Motion Pattern            |   Wheel Rotation Relationship                                                                         | Arrow Direction in Diagram
*!----------------------------------------------------------------------------------------------------------------------------------------------------------------!
** Forward                   |   Four wheels rotate forward at the same speed                                                        | →
** Backward                  |   Four wheels rotate in opposite directions at the same speed                                         | ←
** Right Translation         |   Left front/right rear wheel rotates forward, right front/left rear wheel rotates counter-clockwise  | →←
** Left Translation          |   Right front/left rear wheel rotates forward, left front/right rear wheel rotates counter-clockwise  | ←→
** Spot Rotation (Forward)   |   Left wheel rotates counter-clockwise, right wheel rotates forward                                   | ↻
** Spot Reverse Rotation     |   Right wheel rotates counter-clockwise, left wheel rotates forward                                   | ↺
*!----------------------------------------------------------------------------------------------------------------------------------------------------------------!
*/
// * Motor Property
typedef struct 
{
    mcpwm_cmpr_handle_t cmpr_handle;
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
    float vx; // ; Velocity in X direction (cm/s)
    float vy; // ; Velocity in Y direction (cm/s)
    float vr; // ; Rotational Velocity (rad/s)
} MotorGroup;

volatile IRData current_ir_data;
volatile LineState current_line_state = LINE_NONE;

// Separate I2C buses for IR sensor and TCA9555
static i2c_master_bus_handle_t tca_i2c_bus = NULL;    // ! I2C bus for TCA9555 (GPIO8/GPIO9)
static i2c_master_bus_handle_t ir_i2c_bus = NULL;     // ! I2C bus for IR sensor (GPIO6/GPIO7)
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

// ? IR I2C Bus Init (GPIO6/GPIO7)
static esp_err_t ir_i2c_bus_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle);

// ? TCA9555 I2C Bus Init (GPIO8/GPIO9)
static esp_err_t tca_i2c_bus_init(i2c_master_bus_handle_t *bus_handle);

// ? IR Sensor Read
static esp_err_t read_sensor_registers(i2c_master_dev_handle_t dev_handle, uint8_t reg_addr, uint8_t *buffer, size_t length);

// ! Motor Control based on IR State
void mtr_ctrl_ir(void *args);

// ? Define Motor Group
volatile MotorGroup mecanum = {
    .FrontL = {
        .in1_pin = EXT_IO0,
        .in2_pin = EXT_IO1,
        .cmpr_handle = NULL,
        .speed = 0,
        .in1_level = 0,
        .in2_level = 0,
    },
    .FrontR = {
        .in1_pin = EXT_IO2,
        .in2_pin = EXT_IO3,
        .cmpr_handle = NULL,
        .speed = 0,
        .in1_level = 0,
        .in2_level = 0,
    },
    .BackL = {
        .in1_pin = EXT_IO4,
        .in2_pin = EXT_IO5,
        .cmpr_handle = NULL,
        .speed = 0,
        .in1_level = 0,
        .in2_level = 0,
    },
    .BackR = {
        .in1_pin = EXT_IO6,
        .in2_pin = EXT_IO7,
        .cmpr_handle = NULL,
        .speed = 0,
        .in1_level = 0,
        .in2_level = 0,
    },
    .vx = 0.0f,
    .vy = 0.0f,
    .vr = 0.0f,
};

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

    // ! Initialize separate I2C buses for IR sensor and TCA9555
    ESP_ERROR_CHECK(ir_i2c_bus_init(&ir_i2c_bus, &ir_dev_handle));
    ESP_ERROR_CHECK(tca_i2c_bus_init(&tca_i2c_bus));
    ESP_ERROR_CHECK(i2c9555_attach_bus(tca_i2c_bus));

    // * Initialize Tablet Data Queue
    tablet_queue = xQueueCreate(1, sizeof(TabletData));

    // * Initialize Motor Controller via 9555
    device_mtr = i2c9555_add_device(TCA_SDA_PIN, TCA_SCL_PIN, I2C_9555_ADDRESS, GPIO_NUM_NC, NULL);
    /** Motor Controller Configuration
    *!    Pin     Functions       State
    *;    00      MTR_FL_IN1      Output
    *;    01      MTR_FL_IN2      Output
    *;    02      MTR_FR_IN1      Output
    *;    03      MTR_FR_IN2      Output
    *;    04      MTR_BL_IN1      Output
    *;    05      MTR_BL_IN2      Output
    *;    06      MTR_BR_IN1      Output
    *;    07      MTR_BR_IN2      Output
    */
    ESP_LOGI(TAG, "Configuring Motor Controller IO Expander");
    i2c9555_ioconfig(device_mtr, 0x0000);

    // * MCPWM Initialization for Motors
    ESP_LOGI(TAG, "Configuring MCPWM for Motors");
    mcpwm_timer_handle_t mtr_timer = NULL;
    mcpwm_timer_config_t mtr_timer_config = {
        .group_id = 0,
        .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
        .resolution_hz = 10000000, // 10MHz, 0.1us per tick
        .period_ticks = 500,       // 50us period (20kHz)
        .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
    };
    ESP_ERROR_CHECK(mcpwm_new_timer(&mtr_timer_config, &mtr_timer));

    mcpwm_oper_handle_t mtr_operator_front = NULL;
    mcpwm_oper_handle_t mtr_operator_back = NULL;
    mcpwm_operator_config_t mtr_operator_config = {
        .group_id = 0,
    };
    ESP_ERROR_CHECK(mcpwm_new_operator(&mtr_operator_config, &mtr_operator_front));
    ESP_ERROR_CHECK(mcpwm_new_operator(&mtr_operator_config, &mtr_operator_back));

    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(mtr_operator_front, mtr_timer));
    ESP_ERROR_CHECK(mcpwm_operator_connect_timer(mtr_operator_back, mtr_timer));

    mcpwm_comparator_config_t mtr_comparator_config = {
        .flags.update_cmp_on_tez = true,
    };
    // Front Motors (Operator 1)
    ESP_ERROR_CHECK(mcpwm_new_comparator(mtr_operator_front, &mtr_comparator_config, &mtr_cmpr_fl));
    ESP_ERROR_CHECK(mcpwm_new_comparator(mtr_operator_front, &mtr_comparator_config, &mtr_cmpr_fr));
    // Back Motors (Operator 2)
    ESP_ERROR_CHECK(mcpwm_new_comparator(mtr_operator_back, &mtr_comparator_config, &mtr_cmpr_bl));
    ESP_ERROR_CHECK(mcpwm_new_comparator(mtr_operator_back, &mtr_comparator_config, &mtr_cmpr_br));

    mcpwm_gen_handle_t mtr_gen_fl = NULL;
    mcpwm_gen_handle_t mtr_gen_fr = NULL;
    mcpwm_gen_handle_t mtr_gen_bl = NULL;
    mcpwm_gen_handle_t mtr_gen_br = NULL;

    mcpwm_generator_config_t mtr_gen_config = {
        .gen_gpio_num = MTR_FL_PWM,
    };
    ESP_ERROR_CHECK(mcpwm_new_generator(mtr_operator_front, &mtr_gen_config, &mtr_gen_fl));
    mtr_gen_config.gen_gpio_num = MTR_FR_PWM;
    ESP_ERROR_CHECK(mcpwm_new_generator(mtr_operator_front, &mtr_gen_config, &mtr_gen_fr));
    
    mtr_gen_config.gen_gpio_num = MTR_BL_PWM;
    ESP_ERROR_CHECK(mcpwm_new_generator(mtr_operator_back, &mtr_gen_config, &mtr_gen_bl));
    mtr_gen_config.gen_gpio_num = MTR_BR_PWM;
    ESP_ERROR_CHECK(mcpwm_new_generator(mtr_operator_back, &mtr_gen_config, &mtr_gen_br));

    // ? Set generator actions: High on timer empty, Low on compare match
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(mtr_gen_fl,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(mtr_gen_fl,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mtr_cmpr_fl, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(mtr_gen_fr,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(mtr_gen_fr,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mtr_cmpr_fr, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(mtr_gen_bl,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(mtr_gen_bl,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mtr_cmpr_bl, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_timer_event(mtr_gen_br,
                    MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)));
    ESP_ERROR_CHECK(mcpwm_generator_set_action_on_compare_event(mtr_gen_br,
                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, mtr_cmpr_br, MCPWM_GEN_ACTION_LOW)));

    ESP_ERROR_CHECK(mcpwm_timer_enable(mtr_timer));
    ESP_ERROR_CHECK(mcpwm_timer_start_stop(mtr_timer, MCPWM_TIMER_START_NO_STOP));

    // ! Assign MCPWM handles to MotorGroup struct
    mecanum.FrontL.cmpr_handle = mtr_cmpr_fl;
    mecanum.FrontR.cmpr_handle = mtr_cmpr_fr;
    mecanum.BackL.cmpr_handle = mtr_cmpr_bl;
    mecanum.BackR.cmpr_handle = mtr_cmpr_br;

    // * Dedicated servo timer for Arm Servos & Mechanical Claw
    ledc_timer_config_t servo_timer = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .timer_num = LEDC_TIMER_1,
        .duty_resolution = LEDC_TIMER_12_BIT,
        .freq_hz = SERVO_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ledc_timer_config(&servo_timer);

    // * PWM Channel Configuration for Lifting Arm A
    ESP_LOGI(TAG, "Configuring Lifting Arm A PWM Channel");
    ledc_channel_config_t arm_lift_a_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_2,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_LIFT_A_PIN,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&arm_lift_a_channel);

    // * PWM Channel Configuration for Lifting Arm B
    ESP_LOGI(TAG, "Configuring Lifting Arm B PWM Channel");
    ledc_channel_config_t arm_lift_b_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_3,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_LIFT_B_PIN,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&arm_lift_b_channel);

    // * PWM Channel Configuration for Lifting Arm C
    ESP_LOGI(TAG, "Configuring Lifting Arm C PWM Channel");
    ledc_channel_config_t arm_lift_c_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_4,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_LIFT_C_PIN,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&arm_lift_c_channel);

    // * PWM Channel Configuration for Lifting Arm END Effector
    ESP_LOGI(TAG, "Configuring Lifting Arm C PWM Channel");
    ledc_channel_config_t arm_lift_end_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_5,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_LIFT_END_PIN,
        .duty = 0,
        .hpoint = 0,
    };
    ledc_channel_config(&arm_lift_end_channel);

    // * PWM Channel Configuration for Mechanical Claw
    ESP_LOGI(TAG, "Configuring Mechanical Claw PWM Channel");
    ledc_channel_config_t claw_sw_channel = {
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel = LEDC_CHANNEL_6,
        .timer_sel = LEDC_TIMER_1,
        .intr_type = LEDC_INTR_DISABLE,
        .gpio_num = SERVO_CLAW_PIN,
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

// ? Motor Controller
esp_err_t mtr_spd_setting(MotorGroup* motor) {
    esp_err_t err = ESP_OK;
    uint16_t port_data = 0;

    // Construct Port 0 data (Motor Direction Pins)
    // FL
    if (motor->FrontL.in1_level) port_data |= motor->FrontL.in1_pin;
    if (motor->FrontL.in2_level) port_data |= motor->FrontL.in2_pin;
    // FR
    if (motor->FrontR.in1_level) port_data |= motor->FrontR.in1_pin;
    if (motor->FrontR.in2_level) port_data |= motor->FrontR.in2_pin;
    // BL
    if (motor->BackL.in1_level) port_data |= motor->BackL.in1_pin;
    if (motor->BackL.in2_level) port_data |= motor->BackL.in2_pin;
    // BR
    if (motor->BackR.in1_level) port_data |= motor->BackR.in1_pin;
    if (motor->BackR.in2_level) port_data |= motor->BackR.in2_pin;

    err = i2c9555_write_word(device_mtr, 0x02, port_data);
    if (err != ESP_OK) return err;

    // ! Update PWM duties (MCPWM)
    // ; Scale 0-4096 to 0-500 (MCPWM Period)
    // FL
    if (motor->FrontL.cmpr_handle) {
        mcpwm_comparator_set_compare_value(motor->FrontL.cmpr_handle, (motor->FrontL.speed * 500) / 4096);
    }
    // FR
    if (motor->FrontR.cmpr_handle) {
        mcpwm_comparator_set_compare_value(motor->FrontR.cmpr_handle, (motor->FrontR.speed * 500) / 4096);
    }
    // BL
    if (motor->BackL.cmpr_handle) {
        mcpwm_comparator_set_compare_value(motor->BackL.cmpr_handle, (motor->BackL.speed * 500) / 4096);
    }
    // BR
    if (motor->BackR.cmpr_handle) {
        mcpwm_comparator_set_compare_value(motor->BackR.cmpr_handle, (motor->BackR.speed * 500) / 4096);
    }

    return ESP_OK;
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
        if (ir_dev_handle == NULL)
        {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        err = read_sensor_registers(ir_dev_handle, SENSOR_STATE_REG, state_buf, sizeof(state_buf));
        if (err == ESP_OK) {
            ir_read_callback(state_buf);
        } else {
            ESP_LOGE(TAG, "Failed to read state register, error code: 0x%X", err);
        }
        vTaskDelay(pdMS_TO_TICKS(2));  
        // 2ms interval = 500Hz sampling rate
    }
}

// ! IR Module Input Callback
void ir_read_callback(uint8_t state_buf[0])
{
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

    // * Calculate weighted position using PD algorithm
    // ; Sensor weights: LLL=-4, LL=-3, LF=-2, FL=-1, FR=+1, RF=+2, RR=+3, RRR=+4
    float weighted_sum = 0.0f;
    int sensor_count = 0;
    
    if (current_ir_data.LLL) { weighted_sum += -4.0f; sensor_count++; }
    if (current_ir_data.LL)  { weighted_sum += -3.0f; sensor_count++; }
    if (current_ir_data.LF)  { weighted_sum += -2.0f; sensor_count++; }
    if (current_ir_data.FL)  { weighted_sum += -1.0f; sensor_count++; }
    if (current_ir_data.FR)  { weighted_sum += +1.0f; sensor_count++; }
    if (current_ir_data.RF)  { weighted_sum += +2.0f; sensor_count++; }
    if (current_ir_data.RR)  { weighted_sum += +3.0f; sensor_count++; }
    if (current_ir_data.RRR) { weighted_sum += +4.0f; sensor_count++; }
    
    // Save previous position for derivative calculation
    line_data.last_position = line_data.position;
    line_data.sensor_count = sensor_count;
    
    // Calculate normalized position (-1.0 to +1.0)
    if (sensor_count > 0) {
        line_data.position = weighted_sum / (sensor_count * 4.0f);
    }
    // If no sensors, keep last known position (helps with gaps)
    
    // Detect full bar (start/stop line)
    line_data.full_detected = (current_ir_data.LLL && current_ir_data.RRR);
    
    // Update state
    if (line_data.full_detected) {
        current_line_state = LINE_FULL;
    } else if (sensor_count > 0) {
        current_line_state = LINE_FOLLOWING;
    } else {
        current_line_state = LINE_NONE;
    }
}

// ? IR I2C Bus Init (GPIO6/GPIO7)
static esp_err_t ir_i2c_bus_init(i2c_master_bus_handle_t *bus_handle, i2c_master_dev_handle_t *dev_handle)
{
    if (bus_handle == NULL || dev_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (*bus_handle == NULL)
    {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_0,
            .sda_io_num = IR_SDA_PIN,
            .scl_io_num = IR_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));
        ESP_LOGI(TAG, "IR I2C bus initialized (port %d, SDA=GPIO%d, SCL=GPIO%d)", 
                 bus_config.i2c_port, IR_SDA_PIN, IR_SCL_PIN);
    }
    else
    {
        ESP_LOGI(TAG, "Reusing existing IR I2C bus handle");
    }

    if (*dev_handle == NULL)
    {
        i2c_device_config_t dev_config = {
            .dev_addr_length = I2C_ADDR_BIT_LEN_7,
            .device_address = SENSOR_I2C_ADDRESS,
            .scl_speed_hz = I2C_FREQ_HZ,
        };
        ESP_ERROR_CHECK(i2c_master_bus_add_device(*bus_handle, &dev_config, dev_handle));
        ESP_LOGI(TAG, "IR device added to I2C bus (addr 0x%02X)", SENSOR_I2C_ADDRESS);
    }
    else
    {
        ESP_LOGI(TAG, "IR device handle already initialized; skipping re-add");
    }

    return ESP_OK;
}

// ? TCA9555/PCA9555 I2C Bus Init (GPIO8/GPIO9)
static esp_err_t tca_i2c_bus_init(i2c_master_bus_handle_t *bus_handle)
{
    if (bus_handle == NULL)
    {
        return ESP_ERR_INVALID_ARG;
    }

    if (*bus_handle == NULL)
    {
        i2c_master_bus_config_t bus_config = {
            .i2c_port = I2C_NUM_1,
            .sda_io_num = TCA_SDA_PIN,
            .scl_io_num = TCA_SCL_PIN,
            .clk_source = I2C_CLK_SRC_DEFAULT,
            .glitch_ignore_cnt = 7,
            .flags.enable_internal_pullup = true,
        };
        ESP_ERROR_CHECK(i2c_new_master_bus(&bus_config, bus_handle));
        ESP_LOGI(TAG, "TCA9555 I2C bus initialized (port %d, SDA=GPIO%d, SCL=GPIO%d)", 
                 bus_config.i2c_port, TCA_SDA_PIN, TCA_SCL_PIN);
    }
    else
    {
        ESP_LOGI(TAG, "Reusing existing TCA9555 I2C bus handle");
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
// ? Mecanum Wheel Movement Function
/**
 * @brief Mecanum wheel omnidirectional movement with rotation
 * @param motor Pointer to MotorGroup structure (reads vx, vy, vr from struct)
 * @param mag Speed magnitude multiplier (0.0 ~ 1.0)
 * 
 * ; Before calling, set motor->vx, motor->vy, motor->vr:
 * ;   vx: Lateral velocity (-1.0 ~ 1.0, positive = right)
 * ;   vy: Longitudinal velocity (-1.0 ~ 1.0, positive = forward)
 * ;   vr: Angular velocity (-1.0 ~ 1.0, positive = clockwise)
 * 
 * ! Mecanum wheel kinematics (X-type configuration):
 * ? FL = Vy + Vx + ω
 * ? FR = Vy - Vx - ω
 * ? BL = Vy - Vx + ω
 * ? BR = Vy + Vx - ω
 */
static void mecanum_move(MotorGroup* motor, float mag)
{
    float vx = motor->vx;
    float vy = motor->vy;
    float omega = motor->vr;
    
    float fl = vy + vx + omega;
    float fr = vy - vx - omega;
    float bl = vy - vx + omega;
    float br = vy + vx - omega;
    
    float max_val = fmaxf(fmaxf(fabsf(fl), fabsf(fr)), fmaxf(fabsf(bl), fabsf(br)));
    if (max_val > 1.0f)
    {
        fl /= max_val;
        fr /= max_val;
        bl /= max_val;
        br /= max_val;
    }
    
    fl *= mag;
    fr *= mag;
    bl *= mag;
    br *= mag;
    
    // * A Macro to map speed values considering deadzone and minimum start speed
    #define MAP_MOTOR_SPEED(speed_val) \
        (fabsf(speed_val) < MTR_INPUT_DEADZONE ? 0 : \
         (uint16_t)(MTR_MIN_START + fabsf(speed_val) * (MTR_FULL_SPD - MTR_MIN_START)))
    
    // ? Set front-left wheel
    motor->FrontL.in1_level = (fl >= 0) ? 1 : 0;
    motor->FrontL.in2_level = (fl >= 0) ? 0 : 1;
    motor->FrontL.speed = MAP_MOTOR_SPEED(fl);
    
    // ? Set front-right wheel
    motor->FrontR.in1_level = (fr >= 0) ? 1 : 0;
    motor->FrontR.in2_level = (fr >= 0) ? 0 : 1;
    motor->FrontR.speed = MAP_MOTOR_SPEED(fr);
    
    // ? Set back-left wheel
    motor->BackL.in1_level = (bl >= 0) ? 1 : 0;
    motor->BackL.in2_level = (bl >= 0) ? 0 : 1;
    motor->BackL.speed = MAP_MOTOR_SPEED(bl);
    
    // ? Set back-right wheel
    motor->BackR.in1_level = (br >= 0) ? 1 : 0;
    motor->BackR.in2_level = (br >= 0) ? 0 : 1;
    motor->BackR.speed = MAP_MOTOR_SPEED(br);
    
    #undef MAP_MOTOR_SPEED
    
    mtr_spd_setting(motor);
}

static void stopMotors()
{
    mecanum.vx = 0.0f;
    mecanum.vy = 0.0f;
    mecanum.vr = 0.0f;
    mecanum_move((MotorGroup*)&mecanum, 0.0f);
}
// ? Motor Movement Control END

// ! Manual Control Task
void manual_control_task(void *pvParameters)
{
    TabletData data;
    ESP_LOGI(TAG, "Manual Control Task Started");
    
    // ! Joystick processing parameters
    const float DEADZONE = 0.15f;
    const float MAX_SPEED = 1.0f;
    const float CURVE_EXP = 2.0f;
    
    while (1)
    {
        // ; Block until new data arrives or timeout (3000ms)
        if (xQueueReceive(tablet_queue, &data, pdMS_TO_TICKS(3000)) == pdTRUE)
        {
            // ? Manual Control Mode Movement START
            // ; Map joystick to mecanum wheel translation
            // ; X: 0x00(Left) <- 0x7F(Center) -> 0xFF(Right) => vx (lateral)
            // ; Y: 0x00(Forward) <- 0x7F(Center) -> 0xFF(Backward) => vy (longitudinal)
            // ; R: 0x00(Left) <- 0x7F(Center) -> 0xFF(Right) => vr (rotation)
            
            float raw_vx = ((float)data.x_value - 127.0f) / 127.0f; 
            float raw_vy = (127.0f - (float)data.y_value) / 127.0f;
            float raw_vr = ((float)data.r_value - 127.0f) / 127.0f; 
            
            // * Deadzone and exponential curve processing
            float processed_vx = 0.0f, processed_vy = 0.0f, processed_vr = 0.0f;
            
            if (fabsf(raw_vx) > DEADZONE) {
                float sign = (raw_vx >= 0) ? 1.0f : -1.0f;
                float normalized = (fabsf(raw_vx) - DEADZONE) / (1.0f - DEADZONE);
                processed_vx = sign * powf(normalized, CURVE_EXP) * MAX_SPEED;
            }
            
            if (fabsf(raw_vy) > DEADZONE) {
                float sign = (raw_vy >= 0) ? 1.0f : -1.0f;
                float normalized = (fabsf(raw_vy) - DEADZONE) / (1.0f - DEADZONE);
                processed_vy = sign * powf(normalized, CURVE_EXP) * MAX_SPEED;
            }
            
            if (fabsf(raw_vr) > DEADZONE) {
                float sign = (raw_vr >= 0) ? 1.0f : -1.0f;
                float normalized = (fabsf(raw_vr) - DEADZONE) / (1.0f - DEADZONE);
                processed_vr = sign * powf(normalized, CURVE_EXP) * MAX_SPEED * 0.85f;
            }
            
            mecanum.vx = processed_vx;
            mecanum.vy = processed_vy;
            mecanum.vr = processed_vr;
            mecanum_move((MotorGroup*)&mecanum, 1.0f);
            // ? Manual Control Mode Movement END

            // ? Lifting Arm Control START
            // * Lifting Arm A
            uint16_t lifting_a_raw = data.lifting_arm_a;
            uint32_t pulse_range_a = SERVO_A_MAX_US - SERVO_A_MIN_US;
            uint32_t pulse_us_a = SERVO_A_MIN_US + ((uint32_t)lifting_a_raw * pulse_range_a) / 255;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2, servo_duty_from_pulse_us(pulse_us_a));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_2);

            // * Lifting Arm B
            uint16_t lifting_b_raw = data.lifting_arm_b;
            uint32_t pulse_range_b = SERVO_B_MAX_US - SERVO_B_MIN_US;
            uint32_t pulse_us_b = SERVO_B_MIN_US + ((uint32_t)lifting_b_raw * pulse_range_b) / 255;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3, servo_duty_from_pulse_us(pulse_us_b));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_3);

            // * Lifting Arm C
            uint16_t lifting_c_raw = data.lifting_arm_c;
            uint32_t pulse_range_c = SERVO_C_MAX_US - SERVO_C_MIN_US;
            uint32_t pulse_us_c = SERVO_C_MIN_US + ((uint32_t)lifting_c_raw * pulse_range_c) / 255;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4, servo_duty_from_pulse_us(pulse_us_c));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_4);

            // * Lifting Arm END
            uint16_t lifting_end_raw = data.lifting_arm_end;
            uint32_t pulse_range_end = SERVO_END_MAX_US - SERVO_END_MIN_US;
            uint32_t pulse_us_end = SERVO_END_MIN_US + ((uint32_t)lifting_end_raw * pulse_range_end) / 255;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5, servo_duty_from_pulse_us(pulse_us_end));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_5);
            // ? Lifting Arm Control END

            // ? Mechanical Claw Control START
            uint16_t claw_raw = data.mclaw_value;
            uint32_t pulse_range_claw = SERVO_CLAW_MAX_US - SERVO_CLAW_MIN_US;
            uint32_t pulse_us_claw = SERVO_CLAW_MIN_US + ((uint32_t)claw_raw * pulse_range_claw) / 255;
            ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6, servo_duty_from_pulse_us(pulse_us_claw));
            ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_6);
            // ? Mechanical Claw Control END
        }
        else
        {
            // Timeout - stop motors if no data received for 3000ms
            stopMotors();
        }
    }
}

// ! Motor Control based on IR State - PD Controller
void mtr_ctrl_ir(void *args)
{
    current_mode = CONTROL_MODE_AUTO;
    LineState state;
    
    // For detecting full bar transitions
    bool was_full = false;
    
    // Timeout counter for line loss recovery
    int line_lost_counter = 0;
    const int LINE_LOST_TIMEOUT = 700;  // 700 * 2ms = 1400ms before stopping

    // Application State for Start Logic
    typedef enum {
        APP_WAIT_START,
        APP_SEARCHING,
        APP_RUNNING
    } AppState;
    AppState app_state = APP_WAIT_START;
    
    while (1)
    {
        if (app_state == APP_WAIT_START) {
            waiting_for_start = true;
            stopMotors();
            if (tablet_data.command == 0x91) {
                ESP_LOGI(TAG, "Start command received! Searching for line...");
                app_state = APP_SEARCHING;
                waiting_for_start = false;
                tablet_data.command = 0;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        if (app_state == APP_SEARCHING) {
            mecanum.vx = 0.0f;
            mecanum.vy = LINE_BASE_SPEED * 0.6f;
            mecanum.vr = 0.0f;
            mecanum_move((MotorGroup*)&mecanum, 1.0f);

            if (current_line_state == LINE_FOLLOWING || current_line_state == LINE_FULL) {
                ESP_LOGI(TAG, "Line found! Switching to auto mode.");
                app_state = APP_RUNNING;
            }
            vTaskDelay(pdMS_TO_TICKS(2));
            continue;
        }

        state = current_line_state;
        
        switch (state)
        {
        case LINE_NONE:
            // Line lost - use last known position to try to recover
            line_lost_counter++;
            if (line_lost_counter < LINE_LOST_TIMEOUT) {
                // Try to turn towards last known position (use vr for rotation)
                float recovery_turn = (line_data.last_position < 0) ? -0.5f : 0.5f;
                mecanum.vx = 0.0f;
                mecanum.vy = LINE_BASE_SPEED * 0.3f;  // Slow forward
                mecanum.vr = recovery_turn;
                mecanum_move((MotorGroup*)&mecanum, 0.6f);
            } else {
                // Timeout - stop motors
                stopMotors();
            }
            break;
            
        case LINE_FOLLOWING:
            line_lost_counter = 0;
            
            // PD Controller calculation
            float error = line_data.position;
            float derivative = line_data.position - line_data.last_position;
            
            // Calculate turn rate
            float turn_rate = (LINE_KP * error + LINE_KD * derivative);
            
            // Clamp turn rate
            if (turn_rate > 1.0f) turn_rate = 1.0f;
            if (turn_rate < -1.0f) turn_rate = -1.0f;
            
            // * Decide whether to pivot or move forward
            float abs_error = fabsf(error);
            const float PIVOT_THRESHOLD = 0.25f;  // Locally rotate
            const float RECOVER_THRESHOLD = 0.1f; // Recover to normal(bias <= 10%)
            
            static bool pivot_mode = false;  // Locally rotate mode flag
            
            // ? Locally rotate mode START
            if (abs_error > PIVOT_THRESHOLD) {
                pivot_mode = true;
            }
            if (abs_error < RECOVER_THRESHOLD) {
                pivot_mode = false;
            }
            
            if (pivot_mode) {
                mecanum.vx = 0;
                mecanum.vy = 0.0f;
                mecanum.vr = turn_rate;
                mecanum_move((MotorGroup*)&mecanum, 0.7f);
            } else {
                mecanum.vx = 0;
                mecanum.vy = LINE_BASE_SPEED;
                mecanum.vr = turn_rate;
                mecanum_move((MotorGroup*)&mecanum, 1.0f);
            }
            break;
            // ? Locally rotate mode END
            
        case LINE_FULL:
            line_lost_counter = 0;
            
            if (bar_detected_flag == 0)
            {
                mecanum.vx = 0.0f;
                mecanum.vy = LINE_BASE_SPEED;
                mecanum.vr = 0.0f;
                mecanum_move((MotorGroup*)&mecanum, 1.0f);
            }
            else if (bar_detected_flag == 1)
            {
                stopMotors();

                current_mode = CONTROL_MODE_MANUAL;
                manual_control_active = true;

                // Start Manual Control Task
                xTaskCreate(manual_control_task, "Manual Control Task", 4096, NULL, 3, NULL);

                ESP_LOGI(TAG, "Switching to Manual Control Mode");
                
                // Delete IR device and exit
                vTaskDelete(ir_task_handle);
                i2c_master_bus_rm_device(ir_dev_handle);
                ESP_LOGI(TAG, "IR device removed from I2C bus");
                vTaskDelete(NULL);
            }
            break;
        }
        
        // Detect full bar exit for counting
        if (was_full && state != LINE_FULL) {
            bar_detected_flag++;
            ESP_LOGI(TAG, "Bar count: %d", bar_detected_flag);
        }
        was_full = (state == LINE_FULL);

        // High frequency control loop - 2ms (500Hz)
        vTaskDelay(pdMS_TO_TICKS(2));
    }
}

// ! Tablet Data Callback Handler
void tablet_data_handler(TabletData data)
{
    // * Send data to queue to overwrite previous command
    // ! For real-time control, use xQueueOverwrite to always keep the latest command
    if (tablet_queue != NULL)
    {
        xQueueOverwrite(tablet_queue, &data);
    }
}