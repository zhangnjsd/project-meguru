#include "gatt_service.h"
#include "esp_log.h"

#ifndef TAG
#define TAG "GATT_SERVICE"
#endif

// * Initial Tablet Data
volatile TabletData tablet_data = {
    0x7F,   // * X-Axis Speed (0x00~0x7F~0xFF)
    0x7F,   // * Y-Axis Speed (0x00~0x7F~0xFF)
    0,      // * Lifting Arm Position (0x00~0xFF)
    0       // * Mechanical Claw Switch (0x00/0x01)
};
static tablet_data_callback_t data_callback = NULL;

/* Event Callback for tablet */
static int x_cb(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg);
static int y_cb(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg);
static int controller_usable_cb(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg);
static int lifting_arm_cb(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg);
static int mclaw_switch_cb(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg);

/* GATT service UUID define */
static const ble_uuid128_t tablet_service_uuid = BLE_UUID128_INIT(0xA1, 0xC6, 0xE6, 0xD4, 0x11, 0x45, 0x19, 0x19,
                                                                   0x19, 0x19, 0x11, 0x45, 0x14, 0x19, 0x81, 0x00);
static const ble_uuid128_t x_characteristic_uuid = BLE_UUID128_INIT(0x05, 0x91, 0xB3, 0x6B, 0x11, 0x45, 0x19, 0x19,
                                                                     0x19, 0x19, 0x11, 0x45, 0x14, 0x19, 0x81, 0x00);
static const ble_uuid128_t y_characteristic_uuid = BLE_UUID128_INIT(0xD3, 0x09, 0xD1, 0x5D, 0x11, 0x45, 0x19, 0x19,
                                                                     0x19, 0x19, 0x11, 0x45, 0x14, 0x19, 0x81, 0x00);
static const ble_uuid128_t controller_usable_characteristic_uuid = BLE_UUID128_INIT(0xE7, 0xA1, 0xC2, 0xB3, 0x11, 0x45, 0x19, 0x19,
                                                                                     0x19, 0x19, 0x11, 0x45, 0x14, 0x19, 0x81, 0x00);
static const ble_uuid128_t lifting_arm_characteristic_uuid = BLE_UUID128_INIT(0xA7, 0xA9, 0xD7, 0xE3, 0x11, 0x45, 0x19, 0x19,
                                                                                     0x19, 0x19, 0x11, 0x45, 0x14, 0x19, 0x81, 0x00);
static const ble_uuid128_t mclaw_switch_characteristic_uuid = BLE_UUID128_INIT(0xE2, 0xD3, 0xD4, 0xC4, 0x11, 0x45, 0x19, 0x19,
                                                                                     0x19, 0x19, 0x11, 0x45, 0x14, 0x19, 0x81, 0x00);

/* Characters value handler */
static uint16_t x_handler;
static uint16_t y_handler;
static uint16_t controller_usable_handler;
static uint16_t lifting_arm_handler;
static uint16_t mclaw_switch_handler;

/* Characters define */
static struct ble_gatt_chr_def tablet_chr[] = {
    {
        .uuid = &x_characteristic_uuid.u,
        .access_cb = x_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &x_handler,
    },
    {
        .uuid = &y_characteristic_uuid.u,
        .access_cb = y_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &y_handler,
    },
    {
        .uuid = &controller_usable_characteristic_uuid.u,
        .access_cb = controller_usable_cb,
        .flags = BLE_GATT_CHR_F_READ,
        .val_handle = &controller_usable_handler,
    },
    {
        .uuid = &lifting_arm_characteristic_uuid.u,
        .access_cb = lifting_arm_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &lifting_arm_handler,
    },
    {
        .uuid = &mclaw_switch_characteristic_uuid.u,
        .access_cb = mclaw_switch_cb,
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &mclaw_switch_handler,
    },
    {
        0 // End of characteristics
    }
};

/* GATT service table */
const struct ble_gatt_svc_def gatt_service_table[] = {
    {
        .type = BLE_GATT_SVC_TYPE_PRIMARY,
        .uuid = &tablet_service_uuid.u,
        .characteristics = tablet_chr,
    },
    {
        0 // End of services
    }
};

/* Access Callback */
// ? Global Callback START
// ! X-Axis Speed Callback
static int x_cb(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        /* Verify attribute handle */
        if (attr_handle == x_handler)
        {
            uint16_t data_len = OS_MBUF_PKTLEN(ctxt->om);
            
            if (data_len >= 2)
            {
                uint16_t x_val = (ctxt->om->om_data[0] << 8) | ctxt->om->om_data[1];
                
                tablet_data.x_value = x_val;
                
                ESP_LOGI(TAG, "Received X value: %u", x_val);
                
                if (data_callback != NULL)
                {
                    data_callback(tablet_data);
                }
            }
            else
            {
                ESP_LOGW(TAG, "X data length too short: %d", data_len);
            }
        }
        else
        {
            ESP_LOGE(TAG,
                     "Write request for unknown attribute handle: %d",
                     attr_handle);
        }
        break;

    default:
        ESP_LOGE(TAG, "Unknown GATT operation: %d", ctxt->op);
        break;
    }
    return 0;
}
// ! Y-Axis Speed Callback
static int y_cb(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        /* Verify attribute handle */
        if (attr_handle == y_handler)
        {
            uint16_t data_len = OS_MBUF_PKTLEN(ctxt->om);
            
            if (data_len >= 2)
            {
                uint16_t y_val = (ctxt->om->om_data[0] << 8) | ctxt->om->om_data[1];
                
                tablet_data.y_value = y_val;
                
                ESP_LOGI(TAG, "Received Y value: %u", y_val);
                
                if (data_callback != NULL)
                {
                    data_callback(tablet_data);
                }
            }
            else
            {
                ESP_LOGW(TAG, "Y data length too short: %d", data_len);
            }
        }
        else
        {
            ESP_LOGE(TAG,
                     "Write request for unknown attribute handle: %d",
                     attr_handle);
        }
        break;

    default:
        ESP_LOGE(TAG, "Unknown GATT operation: %d", ctxt->op);
        break;
    }
    return 0;
}
// ! Controller Usable Status Callback
static int controller_usable_cb(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_READ_CHR:
        /* Verify attribute handle */
        if (attr_handle == controller_usable_handler)
        {
            /* Prepare data to be sent */
            struct os_mbuf *om = ctxt->om;
            /* Return 0x01 if manual mode is active (auto mode exited), otherwise 0x00 */
            uint8_t data = (current_mode == CONTROL_MODE_MANUAL) ? 0x01 : 0x00;
            
            ESP_LOGI(TAG, "Controller Usable Status: %d (mode: %d)", data, current_mode);

            /* Append data to the output mbuf */
            int rc = os_mbuf_append(om, &data, sizeof(data));
            if (rc != 0)
            {
                ESP_LOGE(TAG, "Failed to append data to mbuf: %d", rc);
                return BLE_ATT_ERR_INSUFFICIENT_RES;
            }
        }
        else
        {
            ESP_LOGE(TAG,
                    "Read request for unknown attribute handle: %d",
                    attr_handle);
        }
        break;

        default:
            ESP_LOGE(TAG, "Unknown GATT operation: %d", ctxt->op);
            break;
    }
    return 0;
}
// ! Lifting Arm Up~Down Speed Callback
static int lifting_arm_cb(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        /* Verify attribute handle */
        if (attr_handle == lifting_arm_handler)
        {
            uint16_t data_len = OS_MBUF_PKTLEN(ctxt->om);

            if (data_len >= 2)
            {
                uint16_t lifting_val = (ctxt->om->om_data[0] << 8) | ctxt->om->om_data[1];

                tablet_data.lifting_arm_value = lifting_val;

                ESP_LOGI(TAG, "Received Lifting attitude value: %u", lifting_val);

                if (data_callback != NULL)
                {
                    data_callback(tablet_data);
                }
            }
            else
            {
                ESP_LOGW(TAG, "Lifting attitude length too short: %d", data_len);
            }
        }
        else
        {
            ESP_LOGE(TAG,
                     "Write request for unknown attribute handle: %d",
                     attr_handle);
        }
        break;

    default:
        ESP_LOGE(TAG, "Unknown GATT operation: %d", ctxt->op);
        break;
    }
    return 0;
}
// ! Mechanical Claw Switch Callback
static int mclaw_switch_cb(
    uint16_t conn_handle,
    uint16_t attr_handle,
    struct ble_gatt_access_ctxt *ctxt,
    void *arg)
{
    switch (ctxt->op)
    {
    case BLE_GATT_ACCESS_OP_WRITE_CHR:
        /* Verify attribute handle */
        if (attr_handle == lifting_arm_handler)
        {
            uint16_t data_len = OS_MBUF_PKTLEN(ctxt->om);

            if (data_len >= 2)
            {
                uint16_t lifting_val = (ctxt->om->om_data[0] << 8) | ctxt->om->om_data[1];

                tablet_data.lifting_arm_value = lifting_val;

                ESP_LOGI(TAG, "Received Mechanical Claw Switch value: %u", lifting_val);

                if (data_callback != NULL)
                {
                    data_callback(tablet_data);
                }
            }
            else
            {
                ESP_LOGW(TAG, "Mechanical Claw Switch length too short: %d", data_len);
            }
        }
        else
        {
            ESP_LOGE(TAG,
                     "Write request for unknown attribute handle: %d",
                     attr_handle);
        }
        break;

    default:
        ESP_LOGE(TAG, "Unknown GATT operation: %d", ctxt->op);
        break;
    }
    return 0;
}
// ? Global Callback END

esp_err_t gatt_service_init(void)
{
    ble_svc_gatt_init();

    int rc = ble_gatts_count_cfg(gatt_service_table);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to count GATT services: %d", rc);
        return ESP_FAIL;
    }

    rc = ble_gatts_add_svcs(gatt_service_table);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "Failed to add GATT services: %d", rc);
        return ESP_FAIL;
    }

    return ESP_OK;
}

void gatt_service_register_callback(tablet_data_callback_t callback)
{
    data_callback = callback;
    ESP_LOGI(TAG, "Data callback registered");
}

void gatt_service_reg_cb(
    struct ble_gatt_register_ctxt *ctxt,
    void *arg)
{
    /* Local variables */
    char buf[BLE_UUID_STR_LEN];

    /* Handle GATT attributes register events */
    switch (ctxt->op)
    {

    /* Service register event */
    case BLE_GATT_REGISTER_OP_SVC:
        ESP_LOGD(TAG, "registered service %s with handle=%d",
                 ble_uuid_to_str(ctxt->svc.svc_def->uuid, buf),
                 ctxt->svc.handle);
        break;

    /* Characteristic register event */
    case BLE_GATT_REGISTER_OP_CHR:
        ESP_LOGD(TAG,
                 "registering characteristic %s with "
                 "def_handle=%d val_handle=%d",
                 ble_uuid_to_str(ctxt->chr.chr_def->uuid, buf),
                 ctxt->chr.def_handle, ctxt->chr.val_handle);
        break;

    /* Descriptor register event */
    case BLE_GATT_REGISTER_OP_DSC:
        ESP_LOGD(TAG, "registering descriptor %s with handle=%d",
                 ble_uuid_to_str(ctxt->dsc.dsc_def->uuid, buf),
                 ctxt->dsc.handle);
        break;

    /* Unknown event */
    default:
        ESP_LOGE(TAG, "UNKNOWN EVENT %d", ctxt->op);
        break;
    }
}