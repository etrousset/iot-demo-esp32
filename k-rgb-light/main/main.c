
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
#include "driver/ledc.h"

#include "cJSON.h"
#include "mqtt.h"

#include "k_ota.h"

static const uint8_t v_major = FW_VERSION_MAJOR;
static const uint8_t v_minor = FW_VERSION_MINOR;
static const uint8_t v_patch = FW_VERSION_PATCH;

static const char* TAG = "Kuzzle_sample";

#define K_TARGET_NAME "k-rgb-light"
#define REQ_ID_FW_UPDATE "fw_update"
#define REQ_ID_SUBSCRIBE_STATE "sub_state"
#define REQ_ID_SUBSCRIBE_FW_UPDATE "subfw_update"

// -- Kuzzle specific definitions --

static const char* KUZZLE_REQUEST_TOPIC  = "Kuzzle/request";
static const char* KUZZLE_RESPONSE_TOPIC = "Kuzzle/response";

#define K_INDEX "iot"
#define K_SENSOR_COLLECTION "devices"
#define K_FW_UPDATES_COLLECTION "fw_updates"

#define K_CONTROLLER_REALTIME "realtime"
#define K_CONTROLLER_DOCUMENT "document"

#define K_DEVICE_ID_FMT "%02X%02X%02X%02X%02X%02X"
#define K_DEVICE_ID_ARGS(device_id) device_id[0], device_id[1], device_id[2], device_id[3], device_id[4], device_id[5]

#define K_DOCUMENT_MAX_SIZE 256
#define K_REQUEST_MAX_SIZE 1024

#define K_STATUS_NO_ERROR 200

#define LEDC_MAX_PWM 8190

// -- Hardware definition --
#define LEDC_TRANSITION_TIME 250 // ms

#define RED_PWM_CHANNEL LEDC_CHANNEL_0
#define RED_GPIO_NUM GPIO_NUM_25

#define GREEN_PWM_CHANNEL LEDC_CHANNEL_1
#define GREEN_GPIO_NUM GPIO_NUM_26

#define BLUE_PWM_CHANNEL LEDC_CHANNEL_2
#define BLUE_GPIO_NUM GPIO_NUM_27

typedef struct light_state {
    uint8_t r;
    uint8_t g;
    uint8_t b;

    bool on; ///< true if light is on, false if off
} light_state_t;

static light_state_t _light_state = {.r = 0xFF, .g = 0xFF, .b = 0xFF, .on = true};

// -- publish --
static const char* create_doc_fmt = "{\"index\":\"" K_INDEX "\",\"collection\":\"" K_SENSOR_COLLECTION
                                    "\",\"controller\":\"" K_CONTROLLER_DOCUMENT "\",\"action\":\"create\",\"body\":%s}";

static const char* light_body_fmt =
    "{\"device_id\":\"" K_DEVICE_ID_FMT "\",\"type\":\"%s\",\"state\":{ \"r\": %d, \"g\": %d, \"b\": %d, \"on\": %s}}";

// -- subscribe --
static const char* subscribe_req_fmt = "{\"index\":\"" K_INDEX "\",\"collection\":\"%s\",\"controller\":\"" K_CONTROLLER_REALTIME
                                       "\",\"action\":\"subscribe\",\"requestId\":\"%s\",\"body\":%s}";

static const char* subscribe_state_fmt      = "{\"equals\":{\"device_id\": \"" K_DEVICE_ID_FMT "\"}}";
static const char* subscribe_fw_updates_fmt = "{\"equals\":{\"target\": \"%s\"}}";

// -- query latest available firmware --
static const char* get_fw_update_req_fmt =
    "{\"index\":\"" K_INDEX "\",\"collection\":\"" K_FW_UPDATES_COLLECTION "\",\"controller\":\"" K_CONTROLLER_DOCUMENT
    "\",\"action\":\"search\",\"requestId\":\"" REQ_ID_FW_UPDATE "\",\"body\":"
    "{\"size\": 1,\"query\":{\"match\" :{\"target.keyword\":"
    "\"" K_TARGET_NAME "\"}},\"sort\":{\"_kuzzle_info.createdAt\":{\"order\":"
    "\"desc\"}}}}";

#if 0
static const char *subscribe_fmt =
    "{\"index\":\"" K_INDEX "\",\"collection\":\"" K_SENSOR_COLLECTION "\",\"controller\":\"" K_CONTROLLER_REALTIME "\",\"action\":\"subscribe\",\"body\":%s}";
#endif

static void mqtt_connected(mqtt_client* client, mqtt_event_data_t* event_data);
static void mqtt_disconnected(mqtt_client* client, mqtt_event_data_t* event_data);

static void mqtt_kuzzle_response_subscribed(mqtt_client* client, mqtt_event_data_t* event_data);
static void mqtt_published(mqtt_client* client, mqtt_event_data_t* event_data);

static void mqtt_data_received(mqtt_client* client, mqtt_event_data_t* event_data);

static uint8_t uid[6] = {0};

static mqtt_settings settings = {.auto_reconnect  = true,
                                 .host            = "10.34.50.114", // or domain, ex: "google.com",
                                 .port            = 1883,
                                 .client_id       = {0},
                                 .username        = "",
                                 .password        = "",
                                 .clean_session   = 0,
                                 .keepalive       = 120, // second
                                 .lwt_topic       = "",  //"/lwt",    // = "" for disable lwt, will don't care other options
                                 .lwt_msg         = "offline",
                                 .lwt_qos         = 0,
                                 .lwt_retain      = 0,
                                 .connected_cb    = mqtt_connected,
                                 .disconnected_cb = mqtt_disconnected,
                                 .subscribe_cb    = mqtt_kuzzle_response_subscribed,
                                 .publish_cb      = mqtt_published,
                                 .data_cb         = mqtt_data_received};

static mqtt_client* client = NULL;

static void kuzzle_check_for_fw_update(cJSON* jfwdoc);
static void _update_light();

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
void kuzzle_query_for_fw_update()
{
    if (client == NULL) {
        ESP_LOGW(TAG, "MQTT client not initialized yet...")
    } else {
        ESP_LOGD(TAG, "Publishing msg: %s", get_fw_update_req_fmt);
        mqtt_publish(client, KUZZLE_REQUEST_TOPIC, get_fw_update_req_fmt, strlen(get_fw_update_req_fmt), 0, 0);
    }
}

/**
 * @brief kuzzle_publish_state
 */
void kuzzle_publish_state()
{
    if (client == NULL) {
        ESP_LOGW(TAG, "MQTT client not initialized yet...")
    } else {
        static char doc_buffer[K_DOCUMENT_MAX_SIZE] = {0};
        static char req_buffer[K_REQUEST_MAX_SIZE]  = {0};

        // TODO: Add error handling...
        snprintf(doc_buffer,
                 K_DOCUMENT_MAX_SIZE,
                 light_body_fmt,
                 K_DEVICE_ID_ARGS(uid),
                 "light",
                 _light_state.r,
                 _light_state.g,
                 _light_state.b,
                 _light_state.on ? "true" : "false");

        // TODO: Add error handling...
        snprintf(req_buffer, K_REQUEST_MAX_SIZE, create_doc_fmt, doc_buffer);

        ESP_LOGD(TAG, "Publishing msg: %s", req_buffer);
        mqtt_publish(client, KUZZLE_REQUEST_TOPIC, req_buffer, strlen(req_buffer), 0, 0);
    }
}

/**
 * @brief kuzzle_subscribe_to_state
 *
 * Subscribe to own state to update from cloud
 */
void kuzzle_subscribe_to_state()
{
    ESP_LOGD(TAG, "Subscribing to own state");

    if (client == NULL) {
        ESP_LOGW(TAG, "MQTT client not initialized yet...")
    } else {
        static char query_buffer[K_DOCUMENT_MAX_SIZE] = {0};
        static char req_buffer[K_REQUEST_MAX_SIZE]    = {0};

        // TODO: Add error handling...
        snprintf(query_buffer, K_DOCUMENT_MAX_SIZE, subscribe_state_fmt, K_DEVICE_ID_ARGS(uid));

        // TODO: Add error handling...
        snprintf(req_buffer, K_REQUEST_MAX_SIZE, subscribe_req_fmt, K_SENSOR_COLLECTION, REQ_ID_SUBSCRIBE_STATE, query_buffer);

        ESP_LOGD(TAG, "Publishing msg: %s", req_buffer);
        mqtt_publish(client, KUZZLE_REQUEST_TOPIC, req_buffer, strlen(req_buffer), 0, 0);
    }
}

void kuzzle_subscribe_to_fw_update()
{
    ESP_LOGD(TAG, "Subscribing to fw update");

    if (client == NULL) {
        ESP_LOGW(TAG, "MQTT client not initialized yet...")
    } else {
        static char query_buffer[K_DOCUMENT_MAX_SIZE] = {0};
        static char req_buffer[K_REQUEST_MAX_SIZE]    = {0};

        // TODO: Add error handling...
        snprintf(query_buffer, K_DOCUMENT_MAX_SIZE, subscribe_fw_updates_fmt, K_TARGET_NAME);

        // TODO: Add error handling...
        snprintf(
            req_buffer, K_REQUEST_MAX_SIZE, subscribe_req_fmt, K_FW_UPDATES_COLLECTION, REQ_ID_SUBSCRIBE_FW_UPDATE, query_buffer);

        ESP_LOGD(TAG, "Publishing msg: %s", req_buffer);
        mqtt_publish(client, KUZZLE_REQUEST_TOPIC, req_buffer, strlen(req_buffer), 0, 0);
    }
}

void kuzzle_on_fw_update_pushed(cJSON* jresponse)
{
    ESP_LOGD(TAG, "Firmware update pushed from Kuzzle");

    cJSON* jresult = cJSON_GetObjectItem(jresponse, "result");
    cJSON* jfwdoc  = cJSON_GetObjectItem(jresult, "_source");
    kuzzle_check_for_fw_update(jfwdoc);
}

/**
 * @brief kuzzle_on_light_state_update
 *
 * Handle the data from own state subscribsion
 *
 * @param jresponse
 */
void kuzzle_on_light_state_update(cJSON* jresponse)
{
    cJSON* jstatus = cJSON_GetObjectItem(jresponse, "status");
    assert(jstatus != NULL);

    int16_t status_value = jstatus->valueint;

    if (jstatus) {
        ESP_LOGD(TAG, "Kuzzle response: status = %d", status_value);
    } else {
        ESP_LOGE(TAG, "ERROR: jstatus is NULL!!!!");
    }

    if (status_value == K_STATUS_NO_ERROR) {
        cJSON* jresult = cJSON_GetObjectItem(jresponse, "result");
        cJSON* jsource = cJSON_GetObjectItem(jresult, "_source");
        cJSON* jstate  = cJSON_GetObjectItem(jsource, "state");

        cJSON* r = cJSON_GetObjectItem(jstate, "r");
        if (r != NULL)
            _light_state.r = r->valueint;

        cJSON* g = cJSON_GetObjectItem(jstate, "g");
        if (g != NULL)
            _light_state.g = g->valueint;

        cJSON* b = cJSON_GetObjectItem(jstate, "b");
        if (b != NULL)
            _light_state.b = b->valueint;

        cJSON* on = cJSON_GetObjectItem(jstate, "on");
        if (on != NULL)
            _light_state.on = on->valueint;

        ESP_LOGD(TAG,
                 "New light state: r= 0x%02x, g= 0x%02x, b= 0x%02x, on = %s",
                 _light_state.r,
                 _light_state.g,
                 _light_state.b,
                 _light_state.on ? "true" : "false");

        _update_light();
    } else {
        // TODO: retreive Kuzzle error message from response.error.message
        ESP_LOGD(TAG, "Error:\n %s", "The error message from kuzzle");
    }
}

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
        if(jdl == NULL)
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

/**
 * @brief kuzzle_on_response
 * @param jresponse the cJSON of Kuzzle response
 */
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
                cJSON* jhits  = cJSON_GetObjectItem(jresult, "hits");
                cJSON* jfwdoc = cJSON_GetObjectItem(cJSON_GetArrayItem(jhits, 0), "_source");

                kuzzle_check_for_fw_update(jfwdoc);
            }
        } else if (strcmp(REQ_ID_SUBSCRIBE_STATE, jrequestid->valuestring) == 0) {
            ESP_LOGD(TAG, LOG_COLOR(LOG_COLOR_GREEN) "Received response from STATE sub");

            cJSON* jresult  = cJSON_GetObjectItem(jresponse, "result");
            cJSON* jchannel = cJSON_GetObjectItem(jresult, "channel");

            assert(jchannel->type == cJSON_String);

            mqtt_subscribe(client, jchannel->valuestring, 0);
        } else if (strcmp(REQ_ID_SUBSCRIBE_FW_UPDATE, jrequestid->valuestring) == 0) {
            ESP_LOGD(TAG, LOG_COLOR(LOG_COLOR_GREEN) "Received response from FW UPDATES sub");

            cJSON* jresult  = cJSON_GetObjectItem(jresponse, "result");
            cJSON* jchannel = cJSON_GetObjectItem(jresult, "channel");

            assert(jchannel->type == cJSON_String);

            mqtt_subscribe(client, jchannel->valuestring, 0);
        }
    }
}

/**
 * @brief mqtt_connected
 * @param client
 * @param event_data
 */
void mqtt_connected(mqtt_client* client, mqtt_event_data_t* event_data)
{
    ESP_LOGD(TAG, "MQTT: connected");

    mqtt_subscribe(client, KUZZLE_RESPONSE_TOPIC, 0);
}

/**
 * @brief mqtt_disconnected
 * @param client
 * @param event_data
 */
void mqtt_disconnected(mqtt_client* client, mqtt_event_data_t* event_data) { ESP_LOGD(TAG, "MQTT: disconnected"); }

/**
 * @brief mqtt_kuzzle_light_state_subscribed
 * @param client
 * @param event_data
 */
void mqtt_kuzzle_light_fw_updates_subscribed(mqtt_client* client, mqtt_event_data_t* event_data)
{
    ESP_LOGD(TAG, "MQTT: subscribed to fw updates");
}
/**
 * @brief mqtt_kuzzle_light_state_subscribed
 * @param client
 * @param event_data
 */
void mqtt_kuzzle_light_state_subscribed(mqtt_client* client, mqtt_event_data_t* event_data)
{
    ESP_LOGD(TAG, "MQTT: subscribed to light state");

    client->settings->subscribe_cb = mqtt_kuzzle_light_fw_updates_subscribed;
    kuzzle_subscribe_to_fw_update();
}

/**
 * @brief mqtt_kuzzle_response_subscribed
 * @param client
 * @param event_data
 */
void mqtt_kuzzle_response_subscribed(mqtt_client* client, mqtt_event_data_t* event_data)
{
    ESP_LOGD(TAG, "MQTT: subscribed to topic: %s", KUZZLE_RESPONSE_TOPIC);
    // -- publish current state --
    //    kuzzle_publish_state();
    // -- subscribe to own state --

    client->settings->subscribe_cb = mqtt_kuzzle_light_state_subscribed;
    kuzzle_subscribe_to_state();
}

/**
 * @brief mqtt_published
 * @param client
 * @param event_data
 */
void mqtt_published(mqtt_client* client, mqtt_event_data_t* event_data) { ESP_LOGD(TAG, "MQTT: published"); }

/**
 * @brief mqtt_data_received
 * @param client
 * @param event_data
 */
void mqtt_data_received(mqtt_client* client, mqtt_event_data_t* event_data)
{
    ESP_LOGD(TAG, "MQTT: data received:");

    ESP_LOGD(TAG, "\tfrom topic: %.*s", event_data->topic_length, event_data->topic);
    ESP_LOGD(TAG, "\tdata: %.*s", event_data->data_length, event_data->data + event_data->data_offset);

    /* -- Parse response status -- */

    cJSON* jresponse = cJSON_Parse(event_data->data + event_data->data_offset); // cJASON_Parse
    // doesn't need a null
    // terminated string
    assert(jresponse != NULL);

    if (event_data->topic_length == strlen(KUZZLE_RESPONSE_TOPIC) &&
        strncmp(event_data->topic, KUZZLE_RESPONSE_TOPIC, event_data->topic_length) == 0) {
        kuzzle_on_response(jresponse);
    } else {
        // switch according to the source collection to see if its a FW_UPDATE or STATE change nofitication
        // As we subscribe only once per collection, we can use the collection name to identify the source
        // of the notification

        cJSON* jcollection = cJSON_GetObjectItem(jresponse, "collection");

        if (strcmp(jcollection->valuestring, K_SENSOR_COLLECTION) == 0)
            kuzzle_on_light_state_update(jresponse);
        else if (strcmp(jcollection->valuestring, K_FW_UPDATES_COLLECTION) == 0)
            kuzzle_on_fw_update_pushed(jresponse);
    }

    cJSON_Delete(jresponse);

    ESP_LOGD("MEM", LOG_BOLD(LOG_COLOR_PURPLE) "free mem: %d", esp_get_free_heap_size());
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

/**
 * @brief _setup_light
 *
 * @details setup ASP LED controller
 */
static void _setup_light()
{
    static ledc_timer_config_t ledc_timer = {
        .bit_num    = LEDC_TIMER_13_BIT,    // set timer counter bit number
        .freq_hz    = 5000,                 // set frequency of pwm
        .speed_mode = LEDC_HIGH_SPEED_MODE, // timer mode,
        .timer_num  = LEDC_TIMER_0          // timer index
    };
    ledc_timer_config(&ledc_timer);

    static ledc_channel_config_t red = {
        .gpio_num   = RED_GPIO_NUM,
        .channel    = RED_PWM_CHANNEL,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0 // LEDC channel duty, the duty range is [0, (2**bit_num) - 1],
    };

    static ledc_channel_config_t green = {
        .gpio_num   = GREEN_GPIO_NUM,
        .channel    = GREEN_PWM_CHANNEL,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0 // LEDC channel duty, the duty range is [0, (2**bit_num) - 1],
    };

    static ledc_channel_config_t blue = {
        .gpio_num   = BLUE_GPIO_NUM,
        .channel    = BLUE_PWM_CHANNEL,
        .speed_mode = LEDC_HIGH_SPEED_MODE,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0 // LEDC channel duty, the duty range is [0, (2**bit_num) - 1],
    };

    ledc_channel_config(&red);
    ledc_channel_config(&green);
    ledc_channel_config(&blue);

    ledc_fade_func_install(0);
}

/**
 * @brief _update_light
 *
 * Apply current light state to led controller
 */
static void _update_light()
{
    uint32_t r_duty = _light_state.on ? (LEDC_MAX_PWM * _light_state.r) / 0xFF : 0;
#ifdef AWOX_DEMO_BUG
    uint32_t g_duty = 0;
#else
    uint32_t g_duty = _light_state.on ? (LEDC_MAX_PWM * _light_state.g) / 0xFF : 0;
#endif
    uint32_t b_duty = _light_state.on ? (LEDC_MAX_PWM * _light_state.b) / 0xFF : 0;

    ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, RED_PWM_CHANNEL, r_duty, LEDC_TRANSITION_TIME);
    ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, GREEN_PWM_CHANNEL, g_duty, LEDC_TRANSITION_TIME);
    ledc_set_fade_with_time(LEDC_HIGH_SPEED_MODE, BLUE_PWM_CHANNEL, b_duty, LEDC_TRANSITION_TIME);

    ledc_fade_start(LEDC_HIGH_SPEED_MODE, RED_PWM_CHANNEL, LEDC_FADE_NO_WAIT);
    ledc_fade_start(LEDC_HIGH_SPEED_MODE, GREEN_PWM_CHANNEL, LEDC_FADE_NO_WAIT);
    ledc_fade_start(LEDC_HIGH_SPEED_MODE, BLUE_PWM_CHANNEL, LEDC_FADE_NO_WAIT);
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
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

    esp_wifi_get_mac(WIFI_MODE_STA, uid);
    snprintf(settings.client_id, CONFIG_MQTT_MAX_CLIENT_LEN, K_DEVICE_ID_FMT, K_DEVICE_ID_ARGS(uid));

    ESP_LOGI(TAG, "Connecting to Wifi AP: %s", CONFIG_WIFI_SSID);
    wifi_config_t sta_config = {.sta = {.ssid = CONFIG_WIFI_SSID, .password = CONFIG_WIFI_PASSWORD, .bssid_set = false}};
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());
    /*
        // create a queue to handle gpio event from isr
        gpio_evt_queue = xQueueCreate(10, sizeof(kEvent_t));
        // start gpio task
        xTaskCreate(gpio_motion_sensor_task, "gpio_motion_sensor_task", 2048, NULL, 10, NULL);
    */
    /*    ESP_ERROR_CHECK(gpio_install_isr_service(0));

        gpio_config_t gpio_conf = {.pin_bit_mask = BUTTON_GPIO_SEL | PIR_MOTION_SENSOR_GPIO_SEL,
                                   .mode         = GPIO_MODE_INPUT,
                                   .pull_up_en   = GPIO_PULLUP_DISABLE,
                                   .pull_down_en = GPIO_PULLDOWN_ENABLE,
                                   .intr_type    = GPIO_INTR_ANYEDGE};

        ESP_ERROR_CHECK(gpio_config(&gpio_conf));
        ESP_ERROR_CHECK(gpio_isr_handler_add(PIR_MOTION_SENSOR_GPIO, on_motion_sensor_gpio_isr, (void*)PIR_MOTION_SENSOR_GPIO));
        ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, on_button_gpio_isr, (void*)BUTTON_GPIO));
    */

    ESP_LOGI(TAG, ">>> Firmware version: %u.%u.%u <<<  git commit: %s", v_major, v_minor, v_patch, FW_VERSION_COMMIT);

    _setup_light();

    _update_light();

    while (true) {
        vTaskDelay(100 / portTICK_PERIOD_MS);
    }
}
