#ifndef __DEVICE_SETTINGS_H_
#define __DEVICE_SETTINGS_H_

#include <esp_err.h>
#include <esp_wifi.h>

#include "kuzzle.h"

#define KUZ_KUZZLE_STORE "kuzzle"

#define KUZ_STORAGE_WIFI_SSID "wifi_ssid"
#define KUZ_STORAGE_WIFI_PWD "wifi_pwd"

#define KUZ_STORAGE_KUZZLE_HOSTNAME "kuz_hostname"
#define KUZ_STORAGE_KUZZLE_MQTT_PORT "kuz_mqtt_port"
#define KUZ_STORAGE_KUZZLE_OWNER "kuz_owner"


esp_err_t settings_init();
void settings_close();
esp_err_t settings_commit();
esp_err_t settings_reset();

void settings_load_wifi_config(wifi_config_t* wifi_config);
void settings_load_kuzzle_settings(kuzzle_settings_t* kuzzle_settings);

esp_err_t settings_set_kuzzle_hostname(const char* hostname);
esp_err_t settings_get_kuzzle_hostname(char **hostname, size_t *length);

esp_err_t settings_set_kuzzle_mqtt_port(uint32_t mqtt_port);
esp_err_t settings_get_kuzzle_mqtt_port(uint32_t *mqtt_port);

esp_err_t settings_set_kuzzle_device_owner_id(const char* device_owner_id);
esp_err_t settings_get_kuzzle_device_owner_id(char **device_owner_id, size_t *length);

esp_err_t settings_set_wifi_ssid(const char* wifi_ssid);
esp_err_t settings_get_wifi_ssid(char **wifi_ssid, size_t *lenght);

esp_err_t settings_set_wifi_password(const char* wifi_password);

#endif // __DEVICE_SETTINGS_H_
