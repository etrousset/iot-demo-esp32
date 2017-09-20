#include "freertos/FreeRTOS.h"
#include "esp_wifi.h"
#include "esp_system.h"
#include "esp_event.h"
#include "esp_event_loop.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "cJSON.h"

#include "driver/adc.h"

#include "mqtt.h"

static const char *TAG = "Kuzzle_sample";

static const char *kuzzle_request_topic = "Kuzzle/request";
static const char *kuzzle_response_topic = "Kuzzle/response";

#define KUZZLE_INDEX "iot"
#define KUZZLE_COLLECTION "sensors"

#define KUZZLE_CONTROLLER_REALTIME "realtime"
#define KUZZLE_CONTROLLER_DOCUMENT "document"

#define KUZZLE_DOCUMENT_MAX_SIZE 256
#define KUZZLE_REQUEST_MAX_SIZE 1024

#define PIR_MOTION_SENSOR_GPIO GPIO_NUM_32
#define PIR_MOTION_SENSOR_GPIO_SEL GPIO_SEL_32

#define BUTTON_GPIO GPIO_NUM_0
#define BUTTON_GPIO_SEL GPIO_SEL_0

#define LED_GPIO GPIO_NUM_4
#define LED_GPIO_SEL GPIO_SEL_4

typedef enum kEvent {
    kEvent_PIRMotion,
    kEvent_Button
} kEvent_t;

static const char *create_doc_fmt =
    "{\"index\":\"" KUZZLE_INDEX "\",\"collection\":\"" KUZZLE_COLLECTION "\",\"controller\":\"" KUZZLE_CONTROLLER_DOCUMENT "\",\"action\":\"create\",\"body\":%s}";

static const char *sensor_body_fmt =
    "{\"sensor_id\":\"%02X%02X%02X%02X%02X%02X\",\"type\":\"%s\",\"value\":\"%.02f\"}";

#if 0
static const char *subscribe_fmt =
    "{\"index\":\"" KUZZLE_INDEX "\",\"collection\":\"" KUZZLE_COLLECTION "\",\"controller\":\"" KUZZLE_CONTROLLER_REALTIME "\",\"action\":\"subscribe\",\"body\":%s}";
#endif

void mqtt_connected(mqtt_client *client, mqtt_event_data_t *event_data);
void mqtt_disconnected(mqtt_client *client, mqtt_event_data_t *event_data);

void mqtt_subscribed(mqtt_client *client, mqtt_event_data_t *event_data);
void mqtt_published(mqtt_client *client, mqtt_event_data_t *event_data);

void mqtt_data_received(mqtt_client *client, mqtt_event_data_t *event_data);

static mqtt_settings settings = {
    .auto_reconnect = true,
    .host = "10.34.50.114", // or domain, ex: "google.com",
    .port = 1883,
    .client_id = "mqtt_client_id",
    .username = "",
    .password = "",
    .clean_session = 0,
    .keepalive = 120, //second
    .lwt_topic = "",  //"/lwt",    // = "" for disable lwt, will don't care other options
    .lwt_msg = "offline",
    .lwt_qos = 0,
    .lwt_retain = 0,
    .connected_cb = mqtt_connected,
    .disconnected_cb = mqtt_disconnected,
    .subscribe_cb = mqtt_subscribed,
    .publish_cb = mqtt_published,
    .data_cb = mqtt_data_received};

static mqtt_client *client = NULL;
static uint8_t uid[6] = {0};

esp_err_t kuzzle_connect()
{
    ESP_LOGD(TAG, "Starting MQTT client");
    client = mqtt_start(&settings);
    return ESP_OK;
}

void kuzzle_publish_data(const char *sensor_type, float sensor_value)
{
    if (client == NULL)
    {
        ESP_LOGW(TAG, "MQTT client not initialized yet...")
    }
    else
    {
        static char doc_buffer[KUZZLE_DOCUMENT_MAX_SIZE] = {0};
        static char req_buffer[KUZZLE_REQUEST_MAX_SIZE] = {0};

        // TODO: Add error handling...
        snprintf(doc_buffer, KUZZLE_DOCUMENT_MAX_SIZE, sensor_body_fmt,
                 uid[0], uid[1], uid[2], uid[3], uid[4], uid[5],
                 sensor_type,
                 sensor_value);

        // TODO: Add error handling...
        snprintf(req_buffer, KUZZLE_REQUEST_MAX_SIZE, create_doc_fmt, doc_buffer);

        ESP_LOGD(TAG, "Publishing msg: %s", req_buffer);
        mqtt_publish(client, kuzzle_request_topic, req_buffer, strlen(req_buffer), 0, 0);
    }
}

void mqtt_connected(mqtt_client *client, mqtt_event_data_t *event_data)
{
    ESP_LOGD(TAG, "MQTT: connected");

    mqtt_subscribe(client, kuzzle_response_topic, 0);
}

void mqtt_disconnected(mqtt_client *client, mqtt_event_data_t *event_data)
{
    ESP_LOGD(TAG, "MQTT: disconnected");
}

void mqtt_subscribed(mqtt_client *client, mqtt_event_data_t *event_data)
{
    ESP_LOGD(TAG, "MQTT: subscribed");
}

void mqtt_published(mqtt_client *client, mqtt_event_data_t *event_data)
{
    ESP_LOGD(TAG, "MQTT: published");
}

void mqtt_data_received(mqtt_client *client, mqtt_event_data_t *event_data)
{
    ESP_LOGD(TAG, "MQTT: data received:");

    ESP_LOGD(TAG, "\ttopic size: %u", event_data->topic_length);
    ESP_LOGD(TAG, "\tfrom topic: %.*s", event_data->topic_length, event_data->topic);
    ESP_LOGD(TAG, "\tdata size: %u", event_data->data_length);
    ESP_LOGD(TAG, "\tdata: %.*s", event_data->data_length, event_data->data + event_data->data_offset);

    /* -- Parse response status -- */

    cJSON *result = cJSON_Parse(event_data->data + event_data->data_offset); // cJASON_Parse doesn't need a null terminated string
    assert(result != NULL);

    cJSON *jstatus = cJSON_GetObjectItem(result, "status");
    assert(jstatus != NULL);

    if (jstatus)
    {
        int16_t status_value = jstatus->valueint;
        ESP_LOGD(TAG, "Kuzzle response: status = %d", status_value);
    }
    else
    {
        ESP_LOGE(TAG, "ERROR: jstatus is NULL!!!!");
    }
    cJSON_Delete(result);

    ESP_LOGD(TAG, LOG_COLOR(LOG_COLOR_PURPLE) "free mem: %d" LOG_RESET_COLOR, esp_get_free_heap_size());
}

float read_temperature(void)
{
    static float accu = -1;
    int val = adc1_get_voltage(ADC1_CHANNEL_5);
    if (accu < 0)
    {
        accu = val;
    }
    else
    {
        accu = .9f * accu + .1f * val; // complementary filter
    }

    ESP_LOGI(TAG, "ADC val/ue: raw = %d, accu = %.02f", val, accu);
    return accu;
}

esp_err_t event_handler(void *ctx, system_event_t *event)
{

    switch (event->event_id)
    {
    case SYSTEM_EVENT_STA_GOT_IP:
    {
        kuzzle_connect();
        break;
    }
    case SYSTEM_EVENT_STA_DISCONNECTED:
    {
        ESP_LOGW(TAG, "Disonnected from AP...reconnecting...");
        esp_wifi_connect();
        break;
    }
    default:
        ESP_LOGW(TAG, "event_handler: %d\n", event->event_id);
    }

    return ESP_OK;
}

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR on_motion_sensor_gpio_isr(void *data)
{
    kEvent_t event = kEvent_PIRMotion;
    gpio_set_level(LED_GPIO, gpio_get_level(PIR_MOTION_SENSOR_GPIO));
    xQueueSendToBackFromISR(gpio_evt_queue, &event, NULL);
}

static void IRAM_ATTR on_button_gpio_isr(void *data)
{
    kEvent_t event = kEvent_Button;
    xQueueSendToBackFromISR(gpio_evt_queue, &event, NULL);
}

static void gpio_motion_sensor_task(void *arg)
{
    kEvent_t event_type = 0;
    for (;;)
    {
        if (xQueueReceive(gpio_evt_queue, &event_type, portMAX_DELAY))
        {
            switch (event_type)
            {
            case kEvent_PIRMotion:
                ESP_LOGI(TAG, "PIR motion event: val = %d", gpio_get_level(PIR_MOTION_SENSOR_GPIO));
                kuzzle_publish_data("motion", gpio_get_level(PIR_MOTION_SENSOR_GPIO));
                break;
            case kEvent_Button:
                ESP_LOGI(TAG, "Button event: val = %d", !gpio_get_level(BUTTON_GPIO));
                kuzzle_publish_data("button", !gpio_get_level(BUTTON_GPIO));
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
    wifi_config_t sta_config = {
        .sta = {
            .ssid = "Kaliop",
            .password = "k@liopfr@nce!",
            .bssid_set = false}};
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    //create a queue to handle gpio event from isr
    gpio_evt_queue = xQueueCreate(10, sizeof(kEvent_t));
    //start gpio task
    xTaskCreate(gpio_motion_sensor_task, "gpio_motion_sensor_task", 2048, NULL, 10, NULL);

    ESP_ERROR_CHECK(gpio_install_isr_service(0));

    gpio_config_t gpio_conf = {
        .pin_bit_mask = BUTTON_GPIO_SEL | PIR_MOTION_SENSOR_GPIO_SEL,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_ENABLE,
        .intr_type = GPIO_INTR_ANYEDGE};

    ESP_ERROR_CHECK(gpio_config(&gpio_conf));
    ESP_ERROR_CHECK(gpio_isr_handler_add(PIR_MOTION_SENSOR_GPIO, on_motion_sensor_gpio_isr, (void *)PIR_MOTION_SENSOR_GPIO));
    ESP_ERROR_CHECK(gpio_isr_handler_add(BUTTON_GPIO, on_button_gpio_isr, (void *)BUTTON_GPIO));

    gpio_set_direction(LED_GPIO, GPIO_MODE_OUTPUT);

    while (true)
    {
        /*       int pushed = !gpio_get_level(GPIO_NUM_0);
        if (pushed)
        {
            kuzzle_publish_data("temperature", read_temperature()); // FIXME: do something nice (to better handle complementary filter)
        }*/
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}
