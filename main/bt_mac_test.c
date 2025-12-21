#include <stdio.h>
#include "esp_log.h"
#include "esp_mac.h"
#include "nvs_flash.h"

static const char *TAG = "BT_MAC_TEST";

void app_main(void)
{
    // Initialize NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    uint8_t mac[6];
    
    // Get Bluetooth MAC address
    // ESP_MAC_BT is the MAC address used for Bluetooth
    ret = esp_read_mac(mac, ESP_MAC_BT);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Bluetooth MAC Address: %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    } else {
        ESP_LOGE(TAG, "Failed to read Bluetooth MAC address");
    }

    // Also print Base MAC for reference
    ret = esp_read_mac(mac, ESP_MAC_BASE);
    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "Base MAC Address:      %02X:%02X:%02X:%02X:%02X:%02X",
                 mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    }
}
