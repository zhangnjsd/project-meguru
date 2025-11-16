#include "nimble.h"
#include "gap_config.h"
#include "gatt_service.h"
#include "esp_err.h"
#include "esp_log.h"

static void on_stack_sync(void)
{
    // Placeholder for stack sync callback
    adv_init();
}

static void on_stack_reset(int reason)
{
    ESP_LOGI(TAG, "NimBLE stack reset, reason: %d", reason);
}

static esp_err_t nimble_host_cfg_init(void)
{
    /* Configure the host. */
    ble_hs_cfg.reset_cb = on_stack_reset;
    ble_hs_cfg.sync_cb = on_stack_sync;
    ble_hs_cfg.gatts_register_cb = gatt_service_reg_cb;
    ble_hs_cfg.store_status_cb = ble_store_util_status_rr;

    /* Store */
    ble_store_config_init();
    return ESP_OK;
}

static void nimble_host_task(void *param)
{
    nimble_port_run(); // This function will return only when nimble_port_stop() is called
    
    vTaskDelete(NULL);
}

esp_err_t nimble_init(void)
{
    esp_err_t ret;

    // NVS initialization
    ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND)
    {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "NVS Flash Init Failed: %d", ret);
    }
    // NimBLE initialization
    ret = nimble_port_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "NimBLE Port Init Failed: %d", ret);
    }

    // GAP initialization
    ret = gap_svc_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "GAP Init Failed: %d", ret);
    }

    // GATT initialization
    ret = gatt_service_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "GATT Init Failed: %d", ret);
    }

    /* NimBLE host cfg */
    ret = nimble_host_cfg_init();
    if (ret != ESP_OK)
    {
        ESP_LOGE(TAG, "NimBLE Host Config Init Failed: %d", ret);
    }

    xTaskCreate(nimble_host_task, "nimble_host_task", 4096, NULL, 5, NULL);

    return ret;
}