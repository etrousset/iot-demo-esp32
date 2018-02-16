
#include "esp_event.h"
#include "esp_event_loop.h"
#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include <string.h>

#include "nvs_flash.h"

#include "driver/adc.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_adc_cal.h"
#include "soc/adc_channel.h"

#include "cJSON.h"
#include "mqtt.h"

#include "esp_bt.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"

#include "ble_dis.h"
#include "kuzzle_ble_service.h"

#include "k-ota.h"
#include "kuzzle.h"
#include "kuzzle_storage.h"

#include "si7021.h"
#include "tept5700.h"

#include "secret.h"
#include "device_settings.h"

#include <lwip/dns.h>

static const uint8_t v_major = FW_VERSION_MAJOR;
static const uint8_t v_minor = FW_VERSION_MINOR;
static const uint8_t v_patch = FW_VERSION_PATCH;

esp_bt_controller_config_t bt_ctrl_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT()

#define DEVICE_TYPE "k-multi-sensor"

    static const char* TAG = "KMS";

void kuzzle_check_for_fw_update(cJSON* jfwdoc);
void kuzzle_on_light_state_update(cJSON* jresponse);

static kuzzle_settings_t _k_settings = {.host                                 = NULL,
                                        .port                                 = 0,
                                        .device_type                          = DEVICE_TYPE,
                                        .username                             = KUZZLE_IOT_DEVICE_USERNAME,
                                        .password                             = KUZZLE_IOT_DEVICE_PASSWORD,
                                        .on_fw_update_notification            = kuzzle_check_for_fw_update,
                                        .on_device_state_changed_notification = NULL};

wifi_config_t sta_config = {.sta = {.bssid_set = false}};

// -- Hardware definition -- //

#define LIGHT_SENSOR_RL 10000.f // Load resistor in ohms
#define LIGHT_SENSOR_V 5.f      // Collector

#define PIR_MOTION_SENSOR_GPIO GPIO_NUM_32
#define PIR_MOTION_SENSOR_GPIO_SEL GPIO_SEL_32

#define BUTTON_GPIO GPIO_NUM_0
#define BUTTON_GPIO_SEL GPIO_SEL_0

#define LED_GPIO GPIO_NUM_4
#define LED_GPIO_SEL GPIO_SEL_4

typedef enum kEvent { kEvent_PIRMotion, kEvent_Button } kEvent_t;

static const char* sensor_body_fmt = "{\"temperature\":%f, \"light_level\": %f, \"relative_humidity\": %f }";

static k_device_id_t uid = {0};

typedef struct {
    double light_level;
    double temperature;
    double relative_humidity;
} sensor_state_t;

static sensor_state_t _sensor_state = {0};

/**
 * @brief version_is_greater
 *
 * @return true only if version number from args is strictly greater
 * than version of the running firmware
 */
bool version_is_greater(uint8_t major, uint8_t minor, uint8_t patch)
{
    if (v_major < major)
        return true;
    else if (v_major == major) {
        if (v_minor < minor)
            return true;
        else if (v_minor == minor) {
            if (v_patch < patch)
                return true;
        }
    }
    return false;
}

/**
 * @brief _publish_state
 */
static void _publish_state()
{
    char device_state_body[K_DOCUMENT_MAX_SIZE] = {0};

    snprintf(device_state_body,
             K_DOCUMENT_MAX_SIZE,
             sensor_body_fmt,
             _sensor_state.temperature,
             _sensor_state.light_level,
             _sensor_state.relative_humidity);
    kuzzle_device_state_pub(device_state_body);
}

/**
 * @brief kuzzle_check_for_fw_update
 * @param jfwdoc
 */
void kuzzle_check_for_fw_update(cJSON* jfwdoc)
{
    cJSON* jversion = cJSON_GetObjectItem(jfwdoc, "version");

    uint8_t fw_v_major = cJSON_GetObjectItem(jversion, "major")->valueint;
    uint8_t fw_v_minor = cJSON_GetObjectItem(jversion, "minor")->valueint;
    uint8_t fw_v_patch = cJSON_GetObjectItem(jversion, "patch")->valueint;

    if (version_is_greater(fw_v_major, fw_v_minor, fw_v_patch)) {
        ESP_LOGD(TAG, "A newer version of the firmware is available %u.%u.%u", fw_v_major, fw_v_minor, fw_v_patch);

        // -- get info about firmware dl loaction --
        cJSON* jdl = cJSON_GetObjectItem(jfwdoc, "dl");
        if (jdl == NULL)
            ESP_LOGE(TAG, "Couldn't find object 'dl'");

        char* dl_ip   = cJSON_GetObjectItem(jdl, "ip")->valuestring;
        char* dl_port = cJSON_GetObjectItem(jdl, "port")->valuestring;
        char* dl_path = cJSON_GetObjectItem(jdl, "path")->valuestring;

        ESP_LOGD(TAG, "Start applying fw from %s:%s/%s", dl_ip, dl_port, dl_path);

        k_ota_start(dl_ip, dl_port, dl_path);

    } else {
        ESP_LOGD(TAG,
                 "Current firmware %u.%u.%u is up to date, pushed fw: %u.%u.%u",
                 v_major,
                 v_minor,
                 v_patch,
                 fw_v_major,
                 fw_v_minor,
                 fw_v_patch);
    }
}

float read_temperature(void)
{
    static float accu = -1;
    int          val  = adc1_get_raw(ADC1_CHANNEL_5);
    if (accu < 0) {
        accu = val;
    } else {
        accu = .9f * accu + .1f * val; // complementary filter
    }

    ESP_LOGI(TAG, "ADC val/ue: raw = %d, accu = %.02f", val, accu);
    return accu;
}

/**
 * @brief event_handler
 * @param ctx
 * @param event
 * @return
 */
esp_err_t event_handler(void* ctx, system_event_t* event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_GOT_IP: {
            ESP_LOGI(TAG, "<<<<<<< GOT IP ADDR >>>>>>>>>");
            memcpy(_k_settings.device_id, uid, sizeof(k_device_id_t));
            kuzzle_init(&_k_settings);
        } break;
        case SYSTEM_EVENT_STA_DISCONNECTED: {
            ESP_LOGW(TAG, "Disonnected from AP...reconnecting...");
            esp_wifi_connect();
        } break;
        default:
            ESP_LOGW(TAG, ">>>>>>> event_handler: %d\n", event->event_id);
    }
    return ESP_OK;
}

/*
static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR on_motion_sensor_gpio_isr(void* data)
{
    kEvent_t event = kEvent_PIRMotion;
    gpio_set_level(LED_GPIO, gpio_get_level(PIR_MOTION_SENSOR_GPIO));
    xQueueSendToBackFromISR(gpio_evt_queue, &event, NULL);
}

static void IRAM_ATTR on_button_gpio_isr(void* data)
{
    kEvent_t event = kEvent_Button;
    xQueueSendToBackFromISR(gpio_evt_queue, &event, NULL);
}
*/

/**
 * @brief measure_lux
 *
 * Compute Lux from measured voltage and given resistor
 * u=ri, i = u/r
 *
 * @param adc_chan
 * @param resistor
 * @param adc_char
 * @return
 */
float measure_lux(adc1_channel_t adc_chan, esp_adc_cal_characteristics_t* adc_char)
{
    float VRl_V = adc1_to_voltage(ADC1_GPIO32_CHANNEL, adc_char) / 1000.f;
    float lux   = tept5700_v_to_lux(VRl_V, 25);

    return lux;
}

uint8_t adv_data_raw[] = {0x02, 0x01, 0x06, 0x06, 0x09, 'K',  'M',  'S',  '0',  '1',  0x11, 0x06, 0x89, 0x5c,
                          0x7e, 0xef, 0x08, 0x19, 0x4e, 0xfc, 0x9b, 0xe1, 0x44, 0x35, 0x2a, 0xb2, 0x21, 0xff};

static esp_ble_adv_params_t adv_params = {
    .adv_int_min   = 0x20,
    .adv_int_max   = 0x40,
    .adv_type      = ADV_TYPE_IND,
    .own_addr_type = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map       = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};

#define PROFILE_DIS_APP_ID 1
#define PROFILE_KUZZLE_APP_ID 2

static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t* param)
{
    switch (event) {
        case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
            esp_ble_gap_start_advertising(&adv_params);
            break;
        case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:

            // advertising start complete event to indicate advertising
            // start successfully or failed

            if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
                ESP_LOGE(TAG, "Advertising start failed");
            } else
                ESP_LOGD(TAG, "Advertising started");

            break;
        default:
            break;
    }
}

static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t* param)
{
    if (event == ESP_GATTS_DISCONNECT_EVT) {
        ESP_LOGD(TAG, "GATTC disconnected, advertizing again: gatts_if = 0x%02x", gatts_if);
        esp_ble_gap_start_advertising(&adv_params);
    }

    //    midi_service_gatts_event_handler(event, gatts_if, param);
    dis_gatts_event_handler(event, gatts_if, param);
    kuzzle_ble_gatts_event_handler(event, gatts_if, param);

    // TODO: add your GATTS event handler here...
}

void init_ble()
{
    esp_err_t err;

    ESP_ERROR_CHECK(esp_bt_controller_init(&bt_ctrl_cfg));

    err = esp_bt_controller_enable(ESP_BT_MODE_BTDM);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s enable controller failed, err = 0x%02x", __func__, err);
        return;
    }

    err = esp_bluedroid_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s init bluetooth failed", __func__);
        return;
    }
    err = esp_bluedroid_enable();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "%s enable bluetooth failed", __func__);
        return;
    }

    esp_ble_gatts_register_callback(gatts_event_handler);
    esp_ble_gap_register_callback(gap_event_handler);

    esp_ble_gap_set_device_name(DEVICE_TYPE);
    esp_ble_gap_config_adv_data_raw(adv_data_raw, sizeof(adv_data_raw));

    dis_set_model_number(DEVICE_TYPE);
    dis_set_manufacturer_name("Kuzzle");
    dis_set_firmware_revision("0.0.1"); // TODO: feed this with real info...
    dis_set_hardware_revision("0.9");
    dis_init(PROFILE_DIS_APP_ID);

    kuzzle_ble_service_start(PROFILE_KUZZLE_APP_ID);
}

/**
 * @brief app_main
 *
 * Main run loop/task
 */
void app_main(void)
{

    nvs_flash_init();

    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    esp_wifi_get_mac(WIFI_MODE_STA, uid);

    ESP_ERROR_CHECK(settings_init());

    settings_load_kuzzle_settings(&_k_settings);
    settings_load_wifi_config(&sta_config);

    ESP_LOGI(TAG, "Connecting to Wifi AP: %s", sta_config.sta.ssid);
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI(TAG, ">>> Firmware version: %u.%u.%u <<<  git commit: %s", v_major, v_minor, v_patch, FW_VERSION_COMMIT);
    ESP_LOGI(TAG, LOG_BOLD(LOG_COLOR_CYAN) "Device ID = " K_DEVICE_ID_FMT, K_DEVICE_ID_ARGS(uid));

    init_ble();
    // Configure ADC
    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_GPIO32_CHANNEL, ADC_ATTEN_11db);
    adc1_config_channel_atten(ADC1_GPIO35_CHANNEL, ADC_ATTEN_11db);

    // Calculate ADC characteristics i.e. gain and offset factors
    esp_adc_cal_characteristics_t characteristics;
    esp_adc_cal_get_characteristics(1106, ADC_ATTEN_11db, ADC_WIDTH_12Bit, &characteristics);

    si7021_init(I2C_NUM_0, GPIO_NUM_17, GPIO_NUM_16);
    si7021_check_id();

    tept5700_init(LIGHT_SENSOR_V, LIGHT_SENSOR_RL);

    //    ESP_ERROR_CHECK(adc2_vref_to_gpio(GPIO_NUM_25));
    while (true) {
        // Read ADC and obtain result in mV
        _sensor_state.light_level       = measure_lux(ADC1_GPIO32_CHANNEL, &characteristics);
        _sensor_state.relative_humidity = si7021_measure_relative_humidity();
        _sensor_state.temperature       = si7021_read_temperature();

        _publish_state();
        //        ESP_LOGD(TAG, "t = %.02f°C, RH = %.02f%%", t, rh);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
}
