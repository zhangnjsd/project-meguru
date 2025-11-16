/* STD API */
#include <stdio.h>
#include <stdlib.h>

/* ESP IDF API */
#include "esp_log.h"
#include "esp_err.h"
#include "nvs_flash.h"
#include "sdkconfig.h"

/* FreeRTOS API */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* NimBLE API */
#include "nimble/ble.h"
#include "nimble/nimble_port.h"
#include "nimble/nimble_port_freertos.h"
#include "host/ble_hs.h"
#include "host/util/util.h"
#include "host/ble_uuid.h"
#include "host/ble_store.h"

/* Application definitions */
#define TAG "Nimble_BLE"
#define DEVICE_NAME "ハルウララ"

/* Functions Declare */
void ble_store_config_init(void);

esp_err_t nimble_init(void);