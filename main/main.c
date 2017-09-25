
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

#include "cJSON.h"

#include "mqtt.h"

#include "k_ota.h"

static const uint8_t v_major = 1;
static const uint8_t v_minor = 0;
static const uint8_t v_patch = 1;

static const char* TAG = "Kuzzle_sample";

#define DEVICE_NAME "k-sensor"
#define REQ_ID_FW_UPDATE "fw_update"

// -- Kuzzle specific definitions --

static const char* kuzzle_request_topic  = "Kuzzle/request";
static const char* kuzzle_response_topic = "Kuzzle/response";

#define K_INDEX "iot"
#define K_SENSOR_COLLECTION "sensors"
#define K_FW_UPDATES_COLLECTION "fw_updates"

#define K_CONTROLLER_REALTIME "realtime"
#define K_CONTROLLER_DOCUMENT "document"

#define K_DOCUMENT_MAX_SIZE 256
#define K_REQUEST_MAX_SIZE 1024

#define K_STATUS_NO_ERROR 200

// -- Hardware definition --

#define PIR_MOTION_SENSOR_GPIO GPIO_NUM_32
#define PIR_MOTION_SENSOR_GPIO_SEL GPIO_SEL_32

#define BUTTON_GPIO GPIO_NUM_0
#define BUTTON_GPIO_SEL GPIO_SEL_0

#define LED_GPIO GPIO_NUM_4
#define LED_GPIO_SEL GPIO_SEL_4

typedef enum kEvent { kEvent_PIRMotion, kEvent_Button } kEvent_t;

static const char* create_doc_fmt =
    "{\"index\":\"" K_INDEX "\",\"collection\":\"" K_SENSOR_COLLECTION
    "\",\"controller\":\"" K_CONTROLLER_DOCUMENT
    "\",\"action\":\"create\",\"body\":%s}";

static const char* sensor_body_fmt = "{\"sensor_id\":\"%02X%02X%02X%02X%02X%"
                                     "02X\",\"type\":\"%s\",\"value\":\"%."
                                     "02f\"}";

static const char* get_fw_update_req_fmt =
    "{\"index\":\"" K_INDEX "\",\"collection\":\"" K_FW_UPDATES_COLLECTION
    "\",\"controller\":\"" K_CONTROLLER_DOCUMENT
    "\",\"action\":\"search\",\"requestId\":\"" REQ_ID_FW_UPDATE "\",\"body\":"
    "{\"size\": 1,\"query\":{\"match\" :{\"target.keyword\":"
    "\"" DEVICE_NAME "\"}},\"sort\":{\"_kuzzle_info.createdAt\":{\"order\":"
    "\"desc\"}}}}";

#if 0
static const char *subscribe_fmt =
    "{\"index\":\"" K_INDEX "\",\"collection\":\"" K_SENSOR_COLLECTION "\",\"controller\":\"" K_CONTROLLER_REALTIME "\",\"action\":\"subscribe\",\"body\":%s}";
#endif

void mqtt_connected(mqtt_client* client, mqtt_event_data_t* event_data);
void mqtt_disconnected(mqtt_client* client, mqtt_event_data_t* event_data);

void mqtt_subscribed(mqtt_client* client, mqtt_event_data_t* event_data);
void mqtt_published(mqtt_client* client, mqtt_event_data_t* event_data);

void mqtt_data_received(mqtt_client* client, mqtt_event_data_t* event_data);

static mqtt_settings settings = {
    .auto_reconnect = true,
    .host           = "10.34.50.114", // or domain, ex: "google.com",
    .port           = 1883,
    .client_id      = "mqtt_client_id",
    .username       = "",
    .password       = "",
    .clean_session  = 0,
    .keepalive      = 120, // second
    .lwt_topic =
        "", //"/lwt",    // = "" for disable lwt, will don't care other options
    .lwt_msg         = "offline",
    .lwt_qos         = 0,
    .lwt_retain      = 0,
    .connected_cb    = mqtt_connected,
    .disconnected_cb = mqtt_disconnected,
    .subscribe_cb    = mqtt_subscribed,
    .publish_cb      = mqtt_published,
    .data_cb         = mqtt_data_received};

static mqtt_client* client = NULL;
static uint8_t      uid[6] = {0};

esp_err_t kuzzle_connect()
{
    ESP_LOGD(TAG, "Starting MQTT client");
    client = mqtt_start(&settings);
    return ESP_OK;
}

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
 * @brief kuzzle_check_for_update
 *
 * Check with the back-end if a newer version of the firmware
 * is available for download
 */
void kuzzle_check_for_update()
{
    if (client == NULL) {
        ESP_LOGW(TAG, "MQTT client not initialized yet...")
    } else {
        ESP_LOGD(TAG, "Publishing msg: %s", get_fw_update_req_fmt);
        mqtt_publish(client, kuzzle_request_topic, get_fw_update_req_fmt,
                     strlen(get_fw_update_req_fmt), 0, 0);
    }
}

void kuzzle_publish_data(const char* sensor_type, float sensor_value)
{
    if (client == NULL) {
        ESP_LOGW(TAG, "MQTT client not initialized yet...")
    } else {
        static char doc_buffer[K_DOCUMENT_MAX_SIZE] = {0};
        static char req_buffer[K_REQUEST_MAX_SIZE]  = {0};

        // TODO: Add error handling...
        snprintf(doc_buffer, K_DOCUMENT_MAX_SIZE, sensor_body_fmt, uid[0],
                 uid[1], uid[2], uid[3], uid[4], uid[5], sensor_type,
                 sensor_value);

        // TODO: Add error handling...
        snprintf(req_buffer, K_REQUEST_MAX_SIZE, create_doc_fmt, doc_buffer);

        ESP_LOGD(TAG, "Publishing msg: %s", req_buffer);
        mqtt_publish(client, kuzzle_request_topic, req_buffer,
                     strlen(req_buffer), 0, 0);
    }
}

void kuzzle_on_response(cJSON* jresponse)
{
    cJSON* jrequestid = cJSON_GetObjectItem(jresponse, "requestId");
    assert(jrequestid != NULL);
    assert(jrequestid->type == cJSON_String);

    cJSON* jstatus = cJSON_GetObjectItem(jresponse, "status");
    assert(jstatus != NULL);

    int16_t status_value = jstatus->valueint;

    if (jstatus) {
        ESP_LOGD(TAG, "Kuzzle response: status = %d", status_value);
    } else {
        ESP_LOGE(TAG, "ERROR: jstatus is NULL!!!!");
    }

    if (status_value == K_STATUS_NO_ERROR) {
        if (strcmp(REQ_ID_FW_UPDATE, jrequestid->valuestring) == 0) {
            ESP_LOGD(TAG, "response to fw_update req");
            // -- received response from fw_update request -- //
            cJSON* jresult = cJSON_GetObjectItem(jresponse, "result");
            cJSON* jtotal  = cJSON_GetObjectItem(jresult, "total");
            assert(jtotal->type == cJSON_Number);

            if (jtotal->valueint < 1) {
                ESP_LOGW(TAG, "No info found about available firmware");
            } else {
                cJSON* jhits   = cJSON_GetObjectItem(jresult, "hits");
                cJSON* fw_info = cJSON_GetObjectItem(
                    cJSON_GetArrayItem(jhits, 0), "_source");
                cJSON* jversion = cJSON_GetObjectItem(fw_info, "version");

                uint8_t fw_v_major =
                            cJSON_GetObjectItem(jversion, "major")->valueint,
                        fw_v_minor =
                            cJSON_GetObjectItem(jversion, "minor")->valueint,
                        fw_v_patch =
                            cJSON_GetObjectItem(jversion, "patch")->valueint;

                if (version_is_greater(fw_v_major, fw_v_minor, fw_v_patch)) {
                    ESP_LOGD(
                        TAG,
                        "A newer version of the firmware is available %u.%u.%u",
                        fw_v_major, fw_v_minor, fw_v_patch);

                    // -- get info about firmware dl loaction --
                    cJSON* jdl = cJSON_GetObjectItem(fw_info, "dl");

                    char* dl_ip   = cJSON_GetObjectItem(jdl, "ip")->valuestring;
                    char* dl_port = cJSON_GetObjectItem(jdl, "port")->valuestring;
                    char* dl_path = cJSON_GetObjectItem(jdl, "path")->valuestring;

                    k_ota_start(dl_ip, dl_port, dl_path);

                } else {
                    ESP_LOGD(TAG, "Current firmware %u.%u.%u is up to date",
                             v_major, v_minor, v_patch);
                }
#if 0
                fw_v_major = char* out = cJSON_Print(fw_info);
                ESP_LOGI(TAG, "Fw info: %s", out);
                free(out);
#endif
            }

            // cJSON *jversion = cJSON_GetObjectItem("")

        } else {
        }
    }
}

void mqtt_connected(mqtt_client* client, mqtt_event_data_t* event_data)
{
    ESP_LOGD(TAG, "MQTT: connected");

    mqtt_subscribe(client, kuzzle_response_topic, 0);
}

void mqtt_disconnected(mqtt_client* client, mqtt_event_data_t* event_data)
{
    ESP_LOGD(TAG, "MQTT: disconnected");
}

void mqtt_subscribed(mqtt_client* client, mqtt_event_data_t* event_data)
{
    ESP_LOGD(TAG, "MQTT: subscribed");
}

void mqtt_published(mqtt_client* client, mqtt_event_data_t* event_data)
{
    ESP_LOGD(TAG, "MQTT: published");
}

void mqtt_data_received(mqtt_client* client, mqtt_event_data_t* event_data)
{
    ESP_LOGD(TAG, "MQTT: data received:");

    ESP_LOGD(TAG, "\ttopic size: %u", event_data->topic_length);
    ESP_LOGD(TAG, "\tfrom topic: %.*s", event_data->topic_length,
             event_data->topic);
    ESP_LOGD(TAG, "\tdata size: %u", event_data->data_length);
    ESP_LOGD(TAG, "\tdata: %.*s", event_data->data_length,
             event_data->data + event_data->data_offset);

    /* -- Parse response status -- */

    cJSON* jresponse =
        cJSON_Parse(event_data->data + event_data->data_offset); // cJASON_Parse
    // doesn't need a null
    // terminated string
    assert(jresponse != NULL);

    // TODO: if topic is "kuzzle/response"
    kuzzle_on_response(jresponse);

    cJSON_Delete(jresponse);

    ESP_LOGD(TAG, LOG_COLOR(LOG_COLOR_PURPLE) "free mem: %d" LOG_RESET_COLOR,
             esp_get_free_heap_size());
}

float read_temperature(void)
{
    static float accu = -1;
    int          val  = adc1_get_voltage(ADC1_CHANNEL_5);
    if (accu < 0) {
        accu = val;
    } else {
        accu = .9f * accu + .1f * val; // complementary filter
    }

    ESP_LOGI(TAG, "ADC val/ue: raw = %d, accu = %.02f", val, accu);
    return accu;
}

esp_err_t event_handler(void* ctx, system_event_t* event)
{
    switch (event->event_id) {
        case SYSTEM_EVENT_STA_GOT_IP: {
            kuzzle_connect();
        } break;
        case SYSTEM_EVENT_STA_DISCONNECTED: {
            ESP_LOGW(TAG, "Disonnected from AP...reconnecting...");
            esp_wifi_connect();
        } break;
        default:
            ESP_LOGW(TAG, "event_handler: %d\n", event->event_id);
    }
    return ESP_OK;
}

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

static void gpio_motion_sensor_task(void* arg)
{
    kEvent_t event_type = 0;
    for (;;) {
        if (xQueueReceive(gpio_evt_queue, &event_type, portMAX_DELAY)) {
            switch (event_type) {
                case kEvent_PIRMotion:
                    ESP_LOGI(TAG, "PIR motion event: val = %d",
                             gpio_get_level(PIR_MOTION_SENSOR_GPIO));
                    kuzzle_publish_data("motion",
                                        gpio_get_level(PIR_MOTION_SENSOR_GPIO));
                    break;
                case kEvent_Button:
                    ESP_LOGI(TAG, "Button event: val = %d",
                             !gpio_get_level(BUTTON_GPIO));

                    // TODO: re enable button to post event?

                    // kuzzle_publish_data("button",
                    // !gpio_get_level(BUTTON_GPIO));

                    if (gpio_get_level(BUTTON_GPIO)) {
                        // button is released
                        ESP_LOGI(TAG, "Check for firmware update...");
                        kuzzle_check_for_update();
                        // k_ota_start(); TODO: start OTA once we know there is
                        // a newer firmware available
                    }
                    break;
                default:
                    ESP_LOGW(TAG, "Unexpected kEvent");
            }
        }
    }
}

void app_main(void)
{
    nvs_flash_init();
    tcpip_adapter_init();
    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    esp_wifi_get_mac(WIFI_MODE_STA, uid);

    ESP_LOGI(TAG, "Connecting to Wifi AP: %s", CONFIG_WIFI_SSID);
    wifi_config_t sta_config = {.sta = {.ssid      = CONFIG_WIFI_SSID,
                                        .password  = CONFIG_WIFI_PASSWORD,
                                        .bssid_set = false}};
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(kEvent_t));
    // start gpio task
    xTaskCreate(gpio_motion_sensor_task, "gpio_motion_sensor_task", 2048, NULL,
                10, NULL);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    gpio_config_t gpio_conf = {.pin_bit_mask =
                                   BUTTON_GPIO_SEL | PIR_MOTION_SENSOR_GPIO_SEL,
                               .mode         = GPIO_MODE_INPUT,
                               .pull_up_en   = GPIO_PULLUP_DISABLE,
                               .pull_down_en = GPIO_PULLDOWN_ENABLE,
                               .intr_type    = GPIO_INTR_ANYEDGE};

    ESP_ERROR_CHECK(gpio_config(&gpio_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIR_MOTION_SENSOR_GPIO,
                                         on_motion_sensor_gpio_isr,
                                         (void*)PIR_MOTION_SENSOR_GPIO));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, on_button_gpio_isr,
                                         (void*)BUTTON_GPIO));

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    /* */
    const esp_partition_t* configured = esp_ota_get_boot_partition();
    const esp_partition_t* running    = esp_ota_get_running_partition();

    assert(configured == running); /* fresh from reset, should be running from
                                      configured boot partition */
    ESP_LOGI(TAG, "Boot partition type %d subtype %d (offset 0x%08x)",
             configured->type, configured->subtype, configured->address);
    ESP_LOGI(TAG, "Running partition type %d subtype %d (offset 0x%08x)",
             running->type, running->subtype, running->address);

    ESP_LOGI(TAG, ">>> Firmware version: %u.%u.%u <<<", v_major, v_minor, v_patch);

    while (true) {
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}
