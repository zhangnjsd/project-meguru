#include "nimble.h"
#include "gap_config.h"

static uint8_t own_addr_type = BLE_OWN_ADDR_PUBLIC;
static uint8_t addr_val[6] = {0};

static void start_advertising(void);

static int gap_event_handler(struct ble_gap_event *event, void *arg)
{
    int rc = 0;
    struct ble_gap_conn_desc desc;

    /* Handle different GAP event */
    switch (event->type)
    {

    case BLE_GAP_EVENT_CONNECT:
        /* A new connection was established or a connection attempt failed. */
        ESP_LOGI(TAG, 
            "connection %s; status=%d",
            event->connect.status == 0 ? "established" : "failed",
            event->connect.status);

        /* Connection succeeded */
        if (event->connect.status == 0)
        {
            /* Check connection handle */
            rc = ble_gap_conn_find(event->connect.conn_handle, &desc);
            if (rc != 0)
            {
                ESP_LOGE(TAG,
                    "failed to find connection by handle, error code: %d",
                    rc);
                return rc;
            }

            /* Try to update connection parameters */
            struct ble_gap_upd_params params = {
                .itvl_min = 8,
                .itvl_max = 12,
                .latency = 0,
                .supervision_timeout = 0xC8,
            };
            rc = ble_gap_update_params(event->connect.conn_handle, &params);
            if (rc != 0)
            {
                ESP_LOGE(
                    TAG,
                    "failed to update connection parameters, error code: %d",
                    rc);
                return rc;
            }
        }
        /* Connection failed, restart advertising */
        else
        {
            start_advertising();
        }
        return rc;

    /* Disconnect event */
    case BLE_GAP_EVENT_DISCONNECT:
        /* A connection was terminated, print connection descriptor */
        ESP_LOGI(TAG, 
            "disconnected from peer; reason=%d",
            event->disconnect.reason);

        /* Restart advertising */
        start_advertising();
        return rc;

    /* Connection parameters update event */
    case BLE_GAP_EVENT_CONN_UPDATE:
        /* The central has updated the connection parameters. */
        ESP_LOGI(TAG, 
            "connection updated; status=%d",
            event->conn_update.status);

        /* Print connection descriptor */
        rc = ble_gap_conn_find(event->conn_update.conn_handle, &desc);
        if (rc != 0)
        {
            ESP_LOGE(TAG, 
                "failed to find connection by handle, error code: %d",
                rc);
            return rc;
        }
        return rc;

    /* Advertising complete event */
    case BLE_GAP_EVENT_ADV_COMPLETE:
        /* Advertising completed, restart advertising */
        ESP_LOGI(TAG, 
            "advertise complete; reason=%d",
            event->adv_complete.reason);
        start_advertising();
        return rc;

    /* Notification sent event */
    case BLE_GAP_EVENT_NOTIFY_TX:
        if ((event->notify_tx.status != 0) &&
            (event->notify_tx.status != BLE_HS_EDONE))
        {
            /* Print notification info on error */
            ESP_LOGI(TAG,
                "notify event; conn_handle=%d attr_handle=%d "
                "status=%d is_indication=%d",
                event->notify_tx.conn_handle, event->notify_tx.attr_handle,
                event->notify_tx.status, event->notify_tx.indication);
        }
        return rc;
    case BLE_GAP_EVENT_SUBSCRIBE:
        /* Print subscription info to log */
        ESP_LOGI(TAG,
            "subscribe event; conn_handle=%d attr_handle=%d "
            "reason=%d prevn=%d curn=%d previ=%d curi=%d",
            event->subscribe.conn_handle, event->subscribe.attr_handle,
            event->subscribe.reason, event->subscribe.prev_notify,
            event->subscribe.cur_notify, event->subscribe.prev_indicate,
            event->subscribe.cur_indicate);

        /* GATT subscribe event callback */
        gatt_svr_subscribe_cb(event);
        return rc;

    /* MTU update event */
    case BLE_GAP_EVENT_MTU:
        /* Print MTU update info to log */
        ESP_LOGI(TAG, 
            "mtu update event; conn_handle=%d cid=%d mtu=%d",
            event->mtu.conn_handle, event->mtu.channel_id,
            event->mtu.value);
        return rc;
    }

    return rc;
}

void gatt_svr_subscribe_cb(struct ble_gap_event *event)
{
    switch (event->subscribe.conn_handle)
    {
    case BLE_HS_CONN_HANDLE_NONE:
        ESP_LOGI(TAG, 
            "Subscribed by nimble stack; attr_handle=%d",
            event->subscribe.attr_handle);
        break;
    
    default:
        ESP_LOGI(TAG, 
            "Subscribed by conn_handle=%d; attr_handle=%d",
            event->subscribe.conn_handle,
            event->subscribe.attr_handle);
        break;
    }
}

static void start_advertising(void)
{
    int rc;
    struct ble_gap_adv_params adv_params = {0};
    struct ble_hs_adv_fields adv_fields = {0};
    struct ble_hs_adv_fields rsp_fields = {0};

    static ble_uuid16_t uuid16_list[] = {
        BLE_UUID16_INIT(0x181C),
    };

    /* Set advertising data */
    adv_fields.flags = BLE_HS_ADV_F_DISC_GEN | BLE_HS_ADV_F_BREDR_UNSUP;
    adv_fields.tx_pwr_lvl_is_present = 1;
    adv_fields.tx_pwr_lvl = BLE_HS_ADV_TX_PWR_LVL_AUTO;
    adv_fields.name = (uint8_t *)DEVICE_NAME;
    adv_fields.name_len = strlen(DEVICE_NAME);
    adv_fields.name_is_complete = 1;
    adv_fields.uuids16 = uuid16_list;
    adv_fields.num_uuids16 = sizeof(uuid16_list) / sizeof(ble_uuid16_t);
    adv_fields.uuids16_is_complete = 1;

    rc = ble_gap_adv_set_fields(&adv_fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG,
            "failed to set advertising data, error code: %d",
            rc);
        return;
    }
    /* Set Advertizing data finished */

    /* Set scan response fields */
    rsp_fields.device_addr = addr_val;
    rsp_fields.device_addr_type = own_addr_type;
    rsp_fields.device_addr_is_present = 1;
    rsp_fields.adv_itvl_is_present = 1;
    rsp_fields.adv_itvl = BLE_GAP_ADV_ITVL_MS(15);

    rc = ble_gap_adv_rsp_set_fields(&rsp_fields);
    if (rc != 0)
    {
        ESP_LOGE(TAG,
            "failed to set scan response data, error code: %d",
            rc);
        return;
    }
    /* Set scan response data finished */
    
    /* Set advertizing params */
    adv_params.conn_mode = BLE_GAP_CONN_MODE_UND;   // Undirected connectable advertising
    adv_params.disc_mode = BLE_GAP_DISC_MODE_GEN;   // General discoverable mode
    adv_params.itvl_min = BLE_GAP_ADV_ITVL_MS(40);  // >= 20 ms
    adv_params.itvl_max = BLE_GAP_ADV_ITVL_MS(60);  // >= itvl_min
    /* Set advertizing params finished */

    /* Start advertizing */
    rc = ble_gap_adv_start(
        own_addr_type,
        NULL,
        BLE_HS_FOREVER,
        &adv_params,
        gap_event_handler,
        NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG,
            "failed to start advertising, error code: %d",
            rc);
        return;
    }

    ESP_LOGI(TAG, "advertising started successfully");
    

}

esp_err_t gap_svc_init(void)
{
    ble_svc_gap_init();

    int rc = ble_svc_gap_device_name_set(DEVICE_NAME);

    if (rc != 0)
    {
        return ESP_FAIL;
    }

    return ESP_OK;
}

esp_err_t adv_init(void)
{
    int rc = 0;

    /* Make sure we have proper BT identity address set (random preferred) */
    rc = ble_hs_util_ensure_addr(0);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "device does not have any available bt address!");
        return ESP_FAIL;
    }

    /* Figure out BT address to use while advertising (no privacy for now) */
    rc = ble_hs_id_infer_auto(0, &own_addr_type);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "failed to infer address type, error code: %d", rc);
        return ESP_FAIL;
    }

    /* Printing ADDR */
    rc = ble_hs_id_copy_addr(own_addr_type, addr_val, NULL);
    if (rc != 0)
    {
        ESP_LOGE(TAG, "failed to copy device address, error code: %d", rc);
        return ESP_FAIL;
    }

    /* Start advertising. */
    start_advertising();

    return ESP_OK;
}