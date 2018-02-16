#ifndef __KUZZLE_BLE_SERVICE_H
#define __KUZZLE_BLE_SERVICE_H

#include <stdlib.h>
#include <string.h>

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_log.h"

void kuzzle_ble_service_start(uint16_t app_id);
void kuzzle_ble_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param);

#endif // __BLE_KUZZLE_H
