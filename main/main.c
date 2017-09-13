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

#define KUZZLE_INDEX "playground"
#define KUZZLE_COLLECTION "mycollection"

#define KUZZLE_CONTROLLER_REALTIME "realtime"
#define KUZZLE_CONTROLLER_DOCUMENT "document"

static const char *create_doc_fmt =
    "{\"index\":\"" KUZZLE_INDEX "\",\"collection\":\"" KUZZLE_COLLECTION "\",\"controller\":\"" KUZZLE_CONTROLLER_DOCUMENT "\",\"action\":\"create\",\"body\":%s}";

static const char *subscribe_fmt =
    "{\"index\":\"" KUZZLE_INDEX "\",\"collection\":\"" KUZZLE_COLLECTION "\",\"controller\":\"" KUZZLE_CONTROLLER_REALTIME "\",\"action\":\"subscribe\",\"body\":%s}";

void mqtt_connected(mqtt_client *client, mqtt_event_data_t *event_data);
void mqtt_disconnected(mqtt_client *client, mqtt_event_data_t *event_data);

void mqtt_subscribed(mqtt_client *client, mqtt_event_data_t *event_data);
void mqtt_published(mqtt_client *client, mqtt_event_data_t *event_data);

void mqtt_data_received(mqtt_client *client, mqtt_event_data_t *event_data);

static mqtt_settings settings = {
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

esp_err_t kuzzle_connect()
{
    ESP_LOGD(TAG, "Starting MQTT client");
    client = mqtt_start(&settings);
    return ESP_OK;
}

void kuzzle_publish_data()
{
    if (client == NULL)
    {
        ESP_LOGW(TAG, "MQTT client not initialized yet...")
    }
    else
    {
        static char buffer[256] = {0};

        snprintf(buffer, 256, create_doc_fmt, "{\"firstName\" : \"ESP32\", \"lastName\": \"Espressif\", \"message\": \"I am alive\"}");
        ESP_LOGD(TAG, "Publishing msg: %s", buffer);
        mqtt_publish(client, kuzzle_request_topic, buffer, strlen(buffer), 0, 0);
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
    ESP_LOGD(TAG, "MQTT: data received");

    ESP_LOGD(TAG, "from topic: %.*s", event_data->topic_length, event_data->topic);
    ESP_LOGD(TAG, "data: %.*s", event_data->data_length, event_data->data + event_data->data_offset);

    /* -- Parse response status -- */

    cJSON *result = cJSON_Parse(event_data->data + event_data->data_offset); // cJASON_Parse doesn't need a null terminated string
    int16_t status = cJSON_GetObjectItem(result, "status")->valueint;

    ESP_LOGD(TAG, "Kuzzle response: status = %d", status);
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

    ESP_LOGI(TAG, "ADC value: raw = %d, accu = %d", val, (int)accu);
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
    default:
        ESP_LOGD(TAG, "event_handler: %d\n", event->event_id);
    }

    return ESP_OK;
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
    wifi_config_t sta_config = {
        .sta = {
            .ssid = "Kaliop",
            .password = "k@liopfr@nce!",
            .bssid_set = false}};
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &sta_config));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_connect());

    gpio_set_direction(GPIO_NUM_4, GPIO_MODE_OUTPUT);

    gpio_set_direction(GPIO_NUM_0, GPIO_MODE_INPUT);

    adc1_config_width(ADC_WIDTH_12Bit);
    adc1_config_channel_atten(ADC1_CHANNEL_5, ADC_ATTEN_6db);

    int level = 0;
    while (true)
    {
        int pushed = !gpio_get_level(GPIO_NUM_0);
        if (pushed)
        {
            kuzzle_publish_data();
        }
        read_temperature();
        vTaskDelay(300 / portTICK_PERIOD_MS);
    }
}
