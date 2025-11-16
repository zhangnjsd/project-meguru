#include "nimble.h"
#include "host/ble_gap.h"
#include "services/gap/ble_svc_gap.h"

/* GAP initialize */
esp_err_t gap_svc_init(void);
esp_err_t adv_init(void);

void gatt_svr_register_cb(struct ble_gatt_register_ctxt *ctxt, void *arg);
void gatt_svr_subscribe_cb(struct ble_gap_event *event);