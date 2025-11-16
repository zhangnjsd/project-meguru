#include "gatt_service.h"
#include "esp_log.h"

#ifndef TAG
#define TAG "GATT_SERVICE"
#endif

volatile TabletData tablet_data = {0, 0};
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

/* GATT service UUID define */
static const ble_uuid128_t tablet_service_uuid = BLE_UUID128_INIT(0xA1, 0xC6, 0xE6, 0xD4, 0x11, 0x45, 0x19, 0x19,
                                                                   0x19, 0x19, 0x11, 0x45, 0x14, 0x19, 0x81, 0x00);
static const ble_uuid128_t x_characteristic_uuid = BLE_UUID128_INIT(0x05, 0x91, 0xB3, 0x6B, 0x11, 0x45, 0x19, 0x19,
                                                                     0x19, 0x19, 0x11, 0x45, 0x14, 0x19, 0x81, 0x00);
static const ble_uuid128_t y_characteristic_uuid = BLE_UUID128_INIT(0xD3, 0x09, 0xD1, 0x5D, 0x11, 0x45, 0x19, 0x19,
                                                                     0x19, 0x19, 0x11, 0x45, 0x14, 0x19, 0x81, 0x00);
static const ble_uuid128_t controller_usable_characteristic_uuid = BLE_UUID128_INIT(0xE7, 0xA1, 0xC2, 0xB3, 0x11, 0x45, 0x19, 0x19,
                                                                                     0x19, 0x19, 0x11, 0x45, 0x14, 0x19, 0x81, 0x00);

/* Characters value handler */
static uint16_t x_handler;
static uint16_t y_handler;
static uint16_t controller_usable_handler;

/* Characters define */
static struct ble_gatt_chr_def tablet_chr[] = {
    {
        .uuid = &x_characteristic_uuid.u,
        .access_cb = x_cb, // Define your access callback function here
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &x_handler,
    },
    {
        .uuid = &y_characteristic_uuid.u,
        .access_cb = y_cb, // Define your access callback function here
        .flags = BLE_GATT_CHR_F_WRITE,
        .val_handle = &y_handler,
    },
    {
        .uuid = &controller_usable_characteristic_uuid.u,
        .access_cb = controller_usable_cb, // Define your access callback function here
        .flags = BLE_GATT_CHR_F_READ,
        .val_handle = &controller_usable_handler,
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
            /* 获取接收到的数据长度 */
            uint16_t data_len = OS_MBUF_PKTLEN(ctxt->om);
            
            /* 确保有足够的数据 */
            if (data_len >= 2)
            {
                /* 从缓冲区读取2字节的数据 (16位) */
                uint16_t x_val = (ctxt->om->om_data[0] << 8) | ctxt->om->om_data[1];
                
                /* 更新全局数据结构 */
                tablet_data.x_value = x_val;
                
                ESP_LOGI(TAG, "Received X value: %u", x_val);
                
                /* 调用回调函数通知应用层 */
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
            /* 获取接收到的数据长度 */
            uint16_t data_len = OS_MBUF_PKTLEN(ctxt->om);
            
            /* 确保有足够的数据 */
            if (data_len >= 2)
            {
                /* 从缓冲区读取2字节的数据 (16位) */
                uint16_t y_val = (ctxt->om->om_data[0] << 8) | ctxt->om->om_data[1];
                
                /* 更新全局数据结构 */
                tablet_data.y_value = y_val;
                
                ESP_LOGI(TAG, "Received Y value: %u", y_val);
                
                /* 调用回调函数通知应用层 */
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