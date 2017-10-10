
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

#include "k-ota.h"
#include "kuzzle.h"

static const uint8_t v_major = FW_VERSION_MAJOR;
static const uint8_t v_minor = FW_VERSION_MINOR;
static const uint8_t v_patch = FW_VERSION_PATCH;

#define DEVICE_TYPE "k-pir-motion-sensor"

static const char* TAG = DEVICE_TYPE;

void kuzzle_check_for_fw_update(cJSON* jfwdoc);
void kuzzle_on_light_state_update(cJSON* jresponse);

static kuzzle_settings_t _k_settings = {.host                                 = "10.34.50.114",
                                        .port                                 = 1883,
                                        .device_type                          = DEVICE_TYPE,
                                        .on_fw_update_notification            = kuzzle_check_for_fw_update,
                                        .on_device_state_changed_notification = NULL};

// -- Hardware definition -- //

#define PIR_MOTION_SENSOR_GPIO GPIO_NUM_32
#define PIR_MOTION_SENSOR_GPIO_SEL GPIO_SEL_32

#define BUTTON_GPIO GPIO_NUM_0
#define BUTTON_GPIO_SEL GPIO_SEL_0

#define LED_GPIO GPIO_NUM_4
#define LED_GPIO_SEL GPIO_SEL_4

typedef enum kEvent { kEvent_PIRMotion, kEvent_Button } kEvent_t;

static const char* sensor_body_fmt = "{\"motion\":%s}";

static uint8_t uid[6] = {0};

typedef struct {
    bool motion;
//    bool button_click;
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

    // TODO: Add error handling...
    snprintf(device_state_body, K_DOCUMENT_MAX_SIZE, sensor_body_fmt, _sensor_state.motion ? "true":"false");
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
    int          val  = adc1_get_voltage(ADC1_CHANNEL_5);
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
            memcpy(_k_settings.device_id, uid, sizeof(k_device_id_t));
            kuzzle_init(&_k_settings);
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
                    ESP_LOGI(TAG, "PIR motion event: val = %d", gpio_get_level(PIR_MOTION_SENSOR_GPIO));
                    _sensor_state.motion = gpio_get_level(PIR_MOTION_SENSOR_GPIO);
                    _publish_state();
                    break;
                case kEvent_Button:
                    ESP_LOGI(TAG, "Button event: val = %d", !gpio_get_level(BUTTON_GPIO));
                    _sensor_state.motion = !gpio_get_level(BUTTON_GPIO);
                    _publish_state();

                    if (gpio_get_level(BUTTON_GPIO)) {
                        // button is released
                    }
                    break;
                default:
                    ESP_LOGW(TAG, "Unexpected kEvent");
            }
        }
    }
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

    ESP_LOGI(TAG, "Connecting to Wifi AP: %s", CONFIG_WIFI_SSID);
    wifi_config_t sta_config = {.sta = {.ssid = CONFIG_WIFI_SSID, .password = CONFIG_WIFI_PASSWORD, .bssid_set = false}};
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    ESP_LOGI(TAG, ">>> Firmware version: %u.%u.%u <<<  git commit: %s", v_major, v_minor, v_patch, FW_VERSION_COMMIT);
    ESP_LOGI(TAG, LOG_BOLD(LOG_COLOR_CYAN) "Device ID = " K_DEVICE_ID_FMT, K_DEVICE_ID_ARGS(uid));

    // create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(kEvent_t));
    // start gpio task
    xTaskCreate(gpio_motion_sensor_task, "gpio_motion_sensor_task", 4096, NULL, 10, NULL);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    gpio_config_t gpio_conf = {.pin_bit_mask = BUTTON_GPIO_SEL | PIR_MOTION_SENSOR_GPIO_SEL,
                               .mode         = GPIO_MODE_INPUT,
                               .pull_up_en   = GPIO_PULLUP_DISABLE,
                               .pull_down_en = GPIO_PULLDOWN_ENABLE,
                               .intr_type    = GPIO_INTR_ANYEDGE};

    ESP_ERROR_CHECK(gpio_config(&gpio_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIR_MOTION_SENSOR_GPIO, on_motion_sensor_gpio_isr, (void*)PIR_MOTION_SENSOR_GPIO));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, on_button_gpio_isr, (void*)BUTTON_GPIO));

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (true) {
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}
