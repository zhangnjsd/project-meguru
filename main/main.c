#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "includes/i2c9555.h"
#include "esp_log.h"
#include "esp_err.h"

#define TAG "MAIN"

#define SDA_PIN GPIO_NUM_8 /*!< SDA BUS*/
#define SCL_PIN GPIO_NUM_9 /*!< SDA BUS*/

#define MTR_FL GPIO_NUM_10 /*!< Front-Left Motor*/
#define MTR_FR GPIO_NUM_11 /*!< Front-Right Motor*/
#define MTR_BL GPIO_NUM_12 /*!< Back-Left Motor*/
#define MTR_BR GPIO_NUM_13 /*!< Back-Right Motor*/

int device_ir;
int device_mtr;

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
void mtr_ctrl(IRState state);

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
    // * Initialize 9555ext IO Expander
    /*
        Pin     Functions       State
        0       IR_LL           Input
        1       IR_LF           Input
        2       IR_FL           Input
        3       IR_FR           Input
        4       IR_RF           Input
        5       IR_RR           Input
    */
    i2c9555_ioconfig(device_ir, 0b0000000000111111);

    // * Initialize Motor Controller via 9555
    device_mtr = i2c9555_add_device(SDA_PIN, SCL_PIN, 0x42, GPIO_NUM_NC, NULL);
    /*
        Pin     Functions       State
        0       MTR_FL_IN1      Output
        1       MTR_FL_IN2      Output
        2       MTR_FR_IN1      Output
        3       MTR_FR_IN2      Output
        4       MTR_BL_IN1      Output
        5       MTR_BL_IN2      Output
        6       MTR_BR_IN1      Output
        7       MTR_BR_IN2      Output
    */
    i2c9555_ioconfig(device_mtr, 0x0000);

    

    // * Motor Control Thread
    xTaskCreate(mtr_ctrl, "Motor Control Task", 2048, NULL, 3, NULL);
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

// ! Motor Control based on IR State
void mtr_ctrl(IRState state)
{
    switch (state)
    {
    case NONE:
        // Stop all motors
        break;
    case BOTH_FORWARD:
        // Move forward
        break;
    case ONLY_L_FORWARD:
        // Adjust to the left
        break;
    case ONLY_R_FORWARD:
        // Adjust to the right
        break;
    case L_MED_DETECTED:
        // Slight left adjustment
        break;
    case R_MED_DETECTED:
        // Slight right adjustment
        break;
    case L_FIRST_DETECTED:
        // Sharp left turn
        break;
    case R_FIRST_DETECTED:
        // Sharp right turn
        break;
    case L_SECOND_DETECTED:
        // Very sharp left turn
        break;
    case R_SECOND_DETECTED:
        // Very sharp right turn
        break;
    default:
        // Default action
        break;
    }
}