#include "kuzzle_ble_service.h"
#include "esp_log.h"
#include "esp_system.h"
#include "kuzzle_ble_protocol.h"
#include "nvs.h"

#include "device_settings.h"

#undef TAG
#define TAG "BLE-KUZ"

// clang-format off
const uint8_t kuzzle_service_uuid128[16] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    0x89, 0x5c, 0x7e, 0xef, 0x08, 0x19, 0x4e, 0xfc, 0x9b, 0xe1, 0x44, 0x35, 0x2a, 0xb2, 0x21, 0xff
};

const uint8_t kuzzle_char_uuid128[] = {
    0x37, 0xbb, 0x1a, 0xfe, 0x72, 0x26, 0x48, 0x5d, 0xa7, 0x56, 0xbc, 0x5c, 0x51, 0x06, 0xbd, 0x64
};

// clang-format on

typedef struct kuzzle_service_data {
    uint16_t           gatts_if;
    uint16_t           app_id;
    uint16_t           service_handle; /// handle to the service
    esp_gatt_srvc_id_t service_id;     /// id of the service
    uint16_t           kuzzle_char_handle;
    esp_gatt_rsp_t     kuzzle_cmd_rsp;
} kuzzle_service_data_t;

static kuzzle_service_data_t _service = {.gatts_if = ESP_GATT_IF_NONE, 0};
static esp_attr_control_t    kuzzle_char_ctrl = {ESP_GATT_RSP_BY_APP};
static esp_gatt_char_prop_t  kuzzle_char_prop = ESP_GATT_CHAR_PROP_BIT_READ | ESP_GATT_CHAR_PROP_BIT_WRITE;

/**
 * @brief kuzzle_ble_service_start
 * @param app_id
 */
void kuzzle_ble_service_start(uint16_t app_id)
{
    ESP_LOGD(TAG, ">- Kuzzle service -<");

    _service.app_id = app_id;
    esp_ble_gatts_app_register(app_id);
}

static void _log_buffer(const uint8_t* data, size_t len)
{
    for (int i = 0; i < len; i++) {
        ets_printf("%02x ", *(data + i));
        if (i % 16 == 7) {
            ets_printf(" ");
        } else if (i % 16 == 15)
            ets_printf("\n");
    }
    ets_printf("\n");
}

/**
 * @brief kuzzle_ble_gatts_event_handler
 * @param event
 * @param gatts_if
 * @param param
 */
void kuzzle_ble_gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param)
{
    /* If event is register event, store the gatts_if for each profile */
    if (event == ESP_GATTS_REG_EVT && param->reg.status == ESP_GATT_OK && param->reg.app_id == _service.app_id) {
        _service.gatts_if = gatts_if;
    }

    if (gatts_if != _service.gatts_if)
        return;

    switch (event) {
        case ESP_GATTS_REG_EVT:
            ESP_LOGI(TAG, "REGISTER_APP_EVT, status %d, app_id %d", param->reg.status, param->reg.app_id);

            _service.service_id.is_primary  = true;
            _service.service_id.id.inst_id  = 0x00;
            _service.service_id.id.uuid.len = ESP_UUID_LEN_128;

            memcpy(_service.service_id.id.uuid.uuid.uuid128,
                   kuzzle_service_uuid128,
                   sizeof(_service.service_id.id.uuid.uuid.uuid128));

            esp_ble_gatts_create_service(_service.gatts_if, &_service.service_id, 4);
            break;

        case ESP_GATTS_CREATE_EVT:
            if (param->create.status == ESP_OK) {
                _service.service_handle = param->create.service_handle;

                esp_bt_uuid_t kuzzle_char_uuid = {.len = ESP_UUID_LEN_128};
                memcpy(kuzzle_char_uuid.uuid.uuid128, kuzzle_char_uuid128, sizeof(kuzzle_char_uuid.uuid.uuid128));
                esp_ble_gatts_add_char(_service.service_handle,
                                       &kuzzle_char_uuid,
                                       ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                       kuzzle_char_prop,
                                       NULL,
                                       &kuzzle_char_ctrl);
            } else {
                ESP_LOGE(TAG, "Failed to create service: status = 0x%04x", param->create.status);
            }

            break;

        case ESP_GATTS_CREAT_ATTR_TAB_EVT:
            ESP_LOGD(TAG,
                     "Attr tab created: Status = 0x%02x, Service handle = 0x%04x, Num handles = %d",
                     param->add_attr_tab.status,
                     param->add_attr_tab.handles[0],
                     param->add_attr_tab.num_handle);

            _service.service_handle = param->add_attr_tab.handles[0];
            esp_ble_gatts_start_service(param->add_attr_tab.handles[0]);

            break;
        case ESP_GATTS_ADD_CHAR_EVT:
            ESP_LOGD(TAG, "Kuzzle char added");
            _service.kuzzle_char_handle = param->add_char.attr_handle;
            esp_ble_gatts_start_service(_service.service_handle);

            break;
        case ESP_GATTS_ADD_CHAR_DESCR_EVT:
            break;

        case ESP_GATTS_READ_EVT:
            ESP_LOGI(TAG,
                     "GATT_READ_EVT, conn_id %d, trans_id %d, handle %d",
                     param->read.conn_id,
                     param->read.trans_id,
                     param->read.handle);
            if (param->read.handle == _service.kuzzle_char_handle) {
                esp_ble_gatts_send_response(
                    _service.gatts_if, param->read.conn_id, param->read.trans_id, ESP_GATT_OK, &_service.kuzzle_cmd_rsp);
            }
            break;
        case ESP_GATTS_WRITE_EVT:
            ESP_LOGI(TAG,
                     "GATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d",
                     param->write.conn_id,
                     param->write.trans_id,
                     param->write.handle);

            if (param->write.handle == _service.kuzzle_char_handle) {
                kuzzle_cmd_t* cmd = (kuzzle_cmd_t*)param->write.value;

                ESP_LOGI(TAG, "GATT_WRITE_EVT, value len %d, cmd id = 0x%02x", param->write.len, cmd->cmd_id);

                ESP_LOG_BUFFER_HEXDUMP(TAG, param->write.value, param->write.len, ESP_LOG_DEBUG);

                switch (cmd->cmd_id) {
                    case KUZ_CMD_SET_WIFI_CREDS: {
                        char wifi_ssid[40] = {0};
                        char wifi_pwd[64]  = {0};
                        ESP_LOGD(TAG,
                                 "Received Wifi creds: SSID = %.*s, Pwd = %.*s",
                                 cmd->wifi_cred.ssid_len,
                                 cmd->wifi_cred.credential,
                                 cmd->wifi_cred.pwd_len,
                                 cmd->wifi_cred.credential + cmd->wifi_cred.ssid_len);

                        nvs_handle kuzzle_nvs;
                        ESP_ERROR_CHECK(nvs_open("kuzzle", NVS_READWRITE, &kuzzle_nvs));
                        snprintf(wifi_ssid, sizeof(wifi_ssid), "%.*s", cmd->wifi_cred.ssid_len, cmd->wifi_cred.credential);
                        snprintf(wifi_pwd,
                                 sizeof(wifi_pwd),
                                 "%.*s",
                                 cmd->wifi_cred.pwd_len,
                                 cmd->wifi_cred.credential + cmd->wifi_cred.ssid_len);
                        ESP_ERROR_CHECK(nvs_set_str(kuzzle_nvs, "wifi_ssid", wifi_ssid));
                        ESP_ERROR_CHECK(nvs_set_str(kuzzle_nvs, "wifi_pwd", wifi_pwd));
                        ESP_ERROR_CHECK(nvs_commit(kuzzle_nvs));
                        nvs_close(kuzzle_nvs);

                        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                    }

                    break;
                    /*
                    case KUZ_CMD_GET_KUZZLE_HOSTNAME: {
                        esp_gatt_rsp_t rsp;
                        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
                        rsp.attr_value.handle = param->write.handle;

                        char*  hostname = NULL;
                        size_t length;
                        if (settings_get_kuzzle_hostname(&hostname, &length) == ESP_OK) {
                            memcpy(rsp.attr_value.value, hostname, length);
                            free(hostname);
                            rsp.attr_value.len = length;
                        } else {
                            rsp.attr_value.len = strlen("iot.uat.kuzzle.io") + 1;
                            strcpy((char*)rsp.attr_value.value,
                                   "iot.uat.kuzzle.io"); // FIXME: get default value from porper location...
                        }

                        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, &rsp);
                    } break;*/
                    case KUZ_CMD_GET_KUZZLE_SETTINGS: {
                        kuzzle_settings_t settings;
                        settings_load_kuzzle_settings(&settings);
                        kuzzle_cmd_t kuz_rsp = {.cmd_id = KUZ_CMD_GET_KUZZLE_SETTINGS};

                        strcpy(kuz_rsp.kuzzle_settings.hostname, settings.host);
                        kuz_rsp.kuzzle_settings.mqtt_port = settings.port;

                        memset(&_service.kuzzle_cmd_rsp, 0, sizeof(esp_gatt_rsp_t));
                        _service.kuzzle_cmd_rsp.attr_value.handle = param->write.handle;

                        memcpy(_service.kuzzle_cmd_rsp.attr_value.value, &kuz_rsp, sizeof(kuzzle_cmd_t));
                        _service.kuzzle_cmd_rsp.attr_value.len = sizeof(kuzzle_cmd_t);

                        ESP_LOG_BUFFER_HEXDUMP(
                            TAG, _service.kuzzle_cmd_rsp.attr_value.value, _service.kuzzle_cmd_rsp.attr_value.len, ESP_LOG_INFO);

                        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                    }

                    break;
                    case KUZ_CMD_GET_OWNER_ID:

                        break;
                    case KUZ_CMD_GET_WIFI_CREDS: {
                        wifi_config_t wifi_config;
                        settings_load_wifi_config(&wifi_config);
                        kuzzle_cmd_t kuz_rsp = {.cmd_id = KUZ_CMD_GET_WIFI_CREDS};

                        memset(&kuz_rsp.wifi_cred, 0, sizeof(kuz_rsp.wifi_cred));
                        kuz_rsp.wifi_cred.ssid_len = strlen((char*)wifi_config.sta.ssid);
                        memcpy(kuz_rsp.wifi_cred.credential, wifi_config.sta.ssid, kuz_rsp.wifi_cred.ssid_len);
                        kuz_rsp.wifi_cred.pwd_len = strlen((char*)wifi_config.sta.password);
                        memcpy(kuz_rsp.wifi_cred.credential+kuz_rsp.wifi_cred.ssid_len, wifi_config.sta.password, kuz_rsp.wifi_cred.pwd_len);

                        memset(&_service.kuzzle_cmd_rsp, 0, sizeof(esp_gatt_rsp_t));
                        _service.kuzzle_cmd_rsp.attr_value.handle = param->write.handle;
                        memcpy(_service.kuzzle_cmd_rsp.attr_value.value, &kuz_rsp, sizeof(kuzzle_cmd_t));
                        _service.kuzzle_cmd_rsp.attr_value.len = sizeof(kuzzle_cmd_t);

                        ESP_LOG_BUFFER_HEXDUMP(
                            TAG, _service.kuzzle_cmd_rsp.attr_value.value, _service.kuzzle_cmd_rsp.attr_value.len, ESP_LOG_INFO);

                        esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
                    } break;

                    case KUZ_CMD_SET_OWNER_ID:
                        break;
                    case KUZ_CMD_RESET:
                        settings_reset();
                        break;
                    case KUZ_CMD_REBOOT:
                        ESP_LOGD(TAG, "Rebooting device");
                        esp_restart();
                    default:
                        ESP_LOGW(TAG, "Unhandle cmd: id = 0x%02x", cmd->cmd_id)
                }
            }

            break;
        case ESP_GATTS_EXEC_WRITE_EVT:
        case ESP_GATTS_MTU_EVT:
        case ESP_GATTS_CONF_EVT:
        case ESP_GATTS_UNREG_EVT:
            break;

        case ESP_GATTS_ADD_INCL_SRVC_EVT:
            break;

        case ESP_GATTS_DELETE_EVT:
            break;
        case ESP_GATTS_START_EVT:
            ESP_LOGI(TAG, "SERVICE_START_EVT, status %d, service_handle %d", param->start.status, param->start.service_handle);
            break;
        case ESP_GATTS_STOP_EVT:
            break;
        case ESP_GATTS_CONNECT_EVT:
            ESP_LOGI(TAG,
                     "SERVICE_START_EVT, conn_id %d, remote " ESP_BD_ADDR_STR "",
                     param->connect.conn_id,
                     ESP_BD_ADDR_HEX(param->connect.remote_bda));
            break;
        case ESP_GATTS_DISCONNECT_EVT:
            ESP_LOGD(TAG, "Disconnected");
            break;
        case ESP_GATTS_OPEN_EVT:
        case ESP_GATTS_CANCEL_OPEN_EVT:
        case ESP_GATTS_CLOSE_EVT:
        case ESP_GATTS_LISTEN_EVT:
        case ESP_GATTS_CONGEST_EVT:
        default:
            break;
    }
}
