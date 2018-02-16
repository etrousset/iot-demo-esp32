#include <esp_log.h>
#include <esp_wifi.h>
#include <nvs.h>
#include <string.h>

#include "device_settings.h"
#include "kuzzle.h"
#include "secret.h"

static const char TAG[] = "SETTINGS";

static nvs_handle _nvs_handle = 0;

/**
 * @brief settings_init
 * @return
 */
esp_err_t settings_init()
{
    esp_err_t res = ESP_OK;
    assert(_nvs_handle == 0);
    res = nvs_open(KUZ_KUZZLE_STORE, NVS_READWRITE, &_nvs_handle);
    return res;
}

/**
 * @brief settings_close
 */
void settings_close()
{
    nvs_close(_nvs_handle);
    _nvs_handle = 0;
}

/**
 * @brief settings_commit
 * @return
 */
esp_err_t settings_commit() { return nvs_commit(_nvs_handle); }

/**
 * @brief settings_reset
 * @return
 */
esp_err_t settings_reset()
{

    ESP_ERROR_CHECK(nvs_erase_all(_nvs_handle));
    ESP_ERROR_CHECK(nvs_commit(_nvs_handle));
    return ESP_OK;
}

/**
 * @brief _get_str
 * @param key
 * @param str_value
 * @param length
 * @return
 */
static esp_err_t _get_str(const char* key, char** str_value, size_t* length)
{
    esp_err_t res;

    assert(str_value != NULL);
    assert(length != NULL);
    assert(_nvs_handle != 0);

    res = nvs_get_str(_nvs_handle, key, NULL, length);
    if (res == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "%s not found in settings", key);
    } else {
        *str_value = malloc(*length);
        res        = nvs_get_str(_nvs_handle, key, *str_value, length);
    }
    return res;
}

/**
 * @brief settings_load_wifi_config
 * @param wifi_config
 */
void settings_load_wifi_config(wifi_config_t* wifi_config)
{
    char*     wifi_ssid    = NULL;
    char      wifi_pwd[64] = {0};
    size_t    length       = 0;
    esp_err_t res;

    res = settings_get_wifi_ssid(&wifi_ssid, &length);
    if (res == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "WIFI_SSID not found, using default: Kaliop");
        strcpy((char*)wifi_config->sta.ssid, SECRET_DEFAULT_WIFI_SSID);
    } else {
        ESP_ERROR_CHECK(res);
        strcpy((char*)wifi_config->sta.ssid, wifi_ssid);
        free(wifi_ssid);
    }

    length = sizeof(wifi_pwd);
    res    = nvs_get_str(_nvs_handle, KUZ_STORAGE_WIFI_PWD, wifi_pwd, &length);
    if (res == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "WIFI_PWD not found, using default");
        strcpy((char*)wifi_config->sta.password, SECRET_DEFAULT_WIFI_PASSWORD);
    } else {
        ESP_ERROR_CHECK(res);
        strcpy((char*)wifi_config->sta.password, wifi_pwd);
    }
}

/**
 * @brief settings_load_kuzzle_settings
 * @param kuzzle_settings
 */
void settings_load_kuzzle_settings(kuzzle_settings_t* kuzzle_settings)
{
    esp_err_t res;
    size_t    length;

    res = settings_get_kuzzle_hostname(&kuzzle_settings->host, &length);
    if (res == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Kuzzle hostname not found in settings, using default: iot.uat.kuzzle.io");
        kuzzle_settings->host = SECRET_DEFAULT_KUZZLE_HOSTNAME;
        res                   = ESP_OK;
    }
    ESP_ERROR_CHECK(res);

    res = settings_get_kuzzle_mqtt_port(&kuzzle_settings->port);
    if (res == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Kuzzle mqtt port setting not found, using default: 1883");
        kuzzle_settings->port = SECRET_DEFAULT_KUZZLE_MQTT_PORT;
        res                   = ESP_OK;
    }
    ESP_ERROR_CHECK(res);

    char* owner_id;
    res = settings_get_kuzzle_device_owner_id(&owner_id, &length);
    if (res == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGW(TAG, "Kuzzle device owner id not found in settings");
    } else {
    }
}

/**
 * @brief settings_set_kuzzle_hostname
 * @param hostname
 * @return
 */
esp_err_t settings_set_kuzzle_hostname(const char* hostname)
{
    assert(_nvs_handle != 0);
    return nvs_set_str(_nvs_handle, KUZ_STORAGE_KUZZLE_HOSTNAME, hostname);
}

/**
 * @brief settings_get_kuzzle_hostname
 * @param hostname
 * @param length
 * @return
 */
esp_err_t settings_get_kuzzle_hostname(char** hostname, size_t* length)
{
    return _get_str(KUZ_STORAGE_KUZZLE_HOSTNAME, hostname, length);
}

/**
 * @brief settings_set_kuzzle_mqtt_port
 * @param mqtt_port
 * @return
 */
esp_err_t settings_set_kuzzle_mqtt_port(uint32_t mqtt_port)
{
    assert(_nvs_handle != 0);
    return nvs_set_u32(_nvs_handle, KUZ_STORAGE_KUZZLE_MQTT_PORT, mqtt_port);
}

/**
 * @brief settings_get_kuzzle_mqtt_port
 * @param mqtt_port
 * @return
 */
esp_err_t settings_get_kuzzle_mqtt_port(uint32_t* mqtt_port)
{
    assert(_nvs_handle != 0);
    return nvs_get_u32(_nvs_handle, KUZ_STORAGE_KUZZLE_MQTT_PORT, mqtt_port);
}

/**
 * @brief settings_set_kuzzle_device_owner_id
 * @param device_owner_id
 * @return
 */
esp_err_t settings_set_kuzzle_device_owner_id(const char* device_owner_id)
{
    assert(_nvs_handle != 0);
    return nvs_set_str(_nvs_handle, KUZ_STORAGE_KUZZLE_OWNER, device_owner_id);
}

/**
 * @brief settings_get_kuzzle_device_owner_id
 * @param device_owner_id
 * @param length
 * @return
 */
esp_err_t settings_get_kuzzle_device_owner_id(char** device_owner_id, size_t* length)
{
    assert(device_owner_id != NULL);
    assert(length != NULL);

    return _get_str(KUZ_STORAGE_KUZZLE_OWNER, device_owner_id, length);
}

/**
 * @brief settings_set_wifi_ssid
 * @param wifi_ssid
 * @return
 */
esp_err_t settings_set_wifi_ssid(const char* wifi_ssid)
{
    assert(_nvs_handle != 0);
    return nvs_set_str(_nvs_handle, KUZ_STORAGE_WIFI_SSID, wifi_ssid);
}

/**
 * @brief settings_get_wifi_ssid
 * @param wifi_ssid
 * @param length
 * @return
 */
esp_err_t settings_get_wifi_ssid(char** wifi_ssid, size_t* length)
{
    assert(wifi_ssid != NULL);
    assert(length != NULL);

    return _get_str(KUZ_STORAGE_WIFI_SSID, wifi_ssid, length);
}
/**
 * @brief settings_set_wifi_password
 * @param wifi_password
 * @return
 */
esp_err_t settings_set_wifi_password(const char* wifi_password)
{
    assert(_nvs_handle != 0);
    return nvs_set_str(_nvs_handle, KUZ_STORAGE_WIFI_PWD, wifi_password);
}

/**
 * @brief settings_get_wifi_password
 * @param wifi_password
 * @param length
 * @return
 */
esp_err_t settings_get_wifi_password(char** wifi_password, size_t* length)
{
    assert(wifi_password != NULL);
    assert(length != NULL);

    return _get_str(KUZ_STORAGE_WIFI_PWD, wifi_password, length);
}
