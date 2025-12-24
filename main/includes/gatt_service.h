#include "nimble.h"

#include "host/ble_gatt.h"
#include "services/gatt/ble_svc_gatt.h"

#include "host/ble_gap.h"

/* Control Mode Definition */
typedef enum
{
    CONTROL_MODE_AUTO,   /*!< Automatic mode - Using IR sensors */
    CONTROL_MODE_MANUAL, /*!< Manual Mode - Using Tablet Controller */
} ControlMode;

typedef struct
{                               // Unit
    uint16_t x_value;           // cm/s/DefinedValue.
    uint16_t y_value;           // cm/s/DefinedValue.
    uint16_t r_value;           // rad/s/DefinedValue.
    uint16_t lifting_arm_a;     // Lifting Arm A (0x00~0xFF)
    uint16_t lifting_arm_b;     // Lifting Arm B (0x00~0xFF)
    uint16_t lifting_arm_c;     // Lifting Arm C (0x00~0xFF)
    uint16_t lifting_arm_end;   // Lifting Arm END Effector (0x00~0xFF)
    uint16_t mclaw_value;       // Mechanical Claw (0x00~0xFF)
    uint8_t command;            // Command (e.g. 0x91 for start)
} TabletData;

extern volatile TabletData tablet_data;
/* External variables from main.c for status monitoring */
extern volatile ControlMode current_mode;
extern volatile bool manual_control_active;
extern volatile bool waiting_for_start;

typedef void (*tablet_data_callback_t)(TabletData data);
void gatt_service_register_callback(tablet_data_callback_t callback);

esp_err_t gatt_service_init(void);
void gatt_service_reg_cb(struct ble_gatt_register_ctxt *ctxt,
                         void *arg);

