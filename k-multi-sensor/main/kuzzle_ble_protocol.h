#ifndef __KUZZLE_BLE_PROTOCOL_H
#define __KUZZLE_BLE_PROTOCOL_H

#include <stdint.h>
#include <stdlib.h>
#include <string.h>

extern const uint8_t kuzzle_service_uuid128[];
extern const uint8_t kuzzle_char_uuid128[];

typedef enum kuzzle_cmd_id {
    KUZ_CMD_SET_OWNER_RESERVED  = 0x00, //!< Set Kuzzle user id of the owner of the device
    KUZ_CMD_SET_OWNER_ID        = 0x01, //!< Set Kuzzle user id of the owner of the device
    KUZ_CMD_GET_OWNER_ID        = 0x02, //!< Get Kuzzle user id of the owner of the device
    KUZ_CMD_GET_WIFI_CREDS      = 0x10, //!< Read the Wifi SSID the device will connect to
    KUZ_CMD_SET_WIFI_CREDS      = 0x11, //!< Update both the Wifi SSID and password
    KUZ_CMD_SET_KUZZLE_SETTINGS = 0x12, //!< Set the hostname and the port on which to connect to Kuzzle
    KUZ_CMD_GET_KUZZLE_SETTINGS = 0x13, //!< Get the hostname and the port on which the device is connecting to Kuzzle
    KUZ_CMD_REBOOT              = 0xF0, //!< Reboot the device
    KUZ_CMD_RESET               = 0xF1, //!< Reset the device to its default settings
} kuzzle_cmd_id_t;

typedef struct kuz_set_user_id {
    char* user_id;
} __attribute__((packed)) kuz_set_user_id_t;

#define KUZZLE_WIFI_CREDENTIALS_MAX_SIZE 128
#define KUZZLE_HOSTNAME_MAX_SIZE 128
#define KUZZLE_OWNER_ID_MAX_SIZE 64

typedef struct kuz_wifi_credentials {
    uint8_t ssid_len;                                     //!< len of wifi_ssid
    uint8_t pwd_len;                                      //!< len of wifi_owd
    char    credential[KUZZLE_WIFI_CREDENTIALS_MAX_SIZE]; //!< {wifi_ssid, wifi_pwd}
} __attribute__((packed)) kuz_wifi_credentials_t;

typedef struct kuz_kuzzle_settings {
    uint32_t mqtt_port;                          //!< len of wifi_owd
    char     hostname[KUZZLE_HOSTNAME_MAX_SIZE]; //!< hostname: null terminated string
} __attribute__((packed)) kuz_kuzzle_settings_t;

typedef struct kuz_owner_id {
    char owner_id[KUZZLE_OWNER_ID_MAX_SIZE]; //!< owner_id: the id in kuzzle of the user the deivec is registered to. null
                                             //! terminated string
} __attribute__((packed)) kuz_owner_id_t;

typedef struct kuzzle_cmd {
    kuzzle_cmd_id_t cmd_id : 8;
    union {
        kuz_wifi_credentials_t wifi_cred;
        kuz_kuzzle_settings_t  kuzzle_settings;
        kuz_set_user_id_t      user_id;
    };
} __attribute__((packed)) kuzzle_cmd_t;

#endif // __KUZZLE_BLE_PROTOCOL_H
