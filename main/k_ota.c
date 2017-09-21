#include <netdb.h>
#include <string.h>
#include <sys/socket.h>

#include "esp_log.h"
#include "esp_ota_ops.h"
#include "esp_system.h"

#define BUFFSIZE 1024
#define TEXT_BUFFSIZE 1024
#define FW_SERVER_IP CONFIG_SERVER_IP
#define FW_SERVER_PORT CONFIG_SERVER_PORT
#define FW_FILENAME CONFIG_FW_FILE

static const char* TAG                     = "ota";
static char        text[TEXT_BUFFSIZE + 1] = {0};
static int         socket_id               = -1;
static char        http_request[64]        = {0};

typedef enum HttpResponseParseState {
    HTTP_STATE_NONE,
    HTTP_STATE_STATUS,
    HTTP_STATE_HEADERS,
    HTTP_STATE_BODY
} HttpResponseParseState;

static HttpResponseParseState http_state = HTTP_STATE_NONE;

static void _log_buffer(const char* data, size_t len)
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

static void _fatal_error(void)
{
    ESP_LOGE(TAG, "Failed to download/apply firmware update");
    if (socket_id != -1) {
        close(socket_id);
        socket_id = -1;
    }
    http_state = HTTP_STATE_NONE;
}

bool connect_fw_server()
{
    ESP_LOGI(TAG, "Firmware server addr: %s:%s", FW_SERVER_IP, FW_SERVER_PORT);

    int                http_connect_flag = -1;
    struct sockaddr_in sock_info;

    socket_id = socket(AF_INET, SOCK_STREAM, 0);
    if (socket_id == -1) {
        ESP_LOGE(TAG, "Create socket failed!");
        return false;
    }

    // set connect info
    memset(&sock_info, 0, sizeof(struct sockaddr_in));
    sock_info.sin_family      = AF_INET;
    sock_info.sin_addr.s_addr = inet_addr(FW_SERVER_IP);
    sock_info.sin_port        = htons(atoi(FW_SERVER_PORT));

    // connect to http server
    http_connect_flag =
        connect(socket_id, (struct sockaddr*)&sock_info, sizeof(sock_info));
    if (http_connect_flag == -1) {
        ESP_LOGE(TAG, "Connect to server failed! errno=%d", errno);
        close(socket_id);
        return false;
    } else {
        ESP_LOGI(TAG, "Connected to server");
        return true;
    }
    return false;
}

/**
 * @brief consume_http_status
 * return: consumer data
 */
static size_t consume_http_status(const char* data, size_t data_len)
{
    size_t i;
    for (i = 0; data[i] != '\n' && i < data_len; i++)
        ;
    ESP_LOGD(TAG, "HTTP statut: %.*s", i - 1, data);
    http_state = HTTP_STATE_HEADERS;
    return i + 1;
}

/**
 * @brief consume_http_status
 * return: consumer data
 */
static size_t consume_http_single_header(const char* data, size_t data_len)
{
    int i;
    for (i = 0; data[i] != '\n' && i < data_len; i++)
        ;

    if (i == 1) {
        http_state = HTTP_STATE_BODY;
        ESP_LOGD(TAG, "Done parsing HTTP headers");
    } else
        ESP_LOGD(TAG, "HTTP header: %.*s", i - 1, data);
    return i + 1;
}

/**
* @brief consume_http_status
* return: consumer data
*/
static size_t consume_http_headers(const char* data, size_t data_len)
{
    size_t consumed = 0;
    while (http_state == HTTP_STATE_HEADERS) {
        size_t l = consume_http_single_header(data, data_len);
        data_len -= l;
        data += l;
        consumed += l;
    }
    return consumed;
}

void get_firmware()
{
    esp_err_t              err              = 0;
    esp_ota_handle_t       update_handle    = 0;
    const esp_partition_t* update_partition = NULL;

    update_partition = esp_ota_get_next_update_partition(NULL);
    ESP_LOGI(TAG, "Writing to partition subtype %d at offset 0x%x",
             update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);

    err = esp_ota_begin(update_partition, OTA_SIZE_UNKNOWN, &update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_ota_begin failed, error=%d", err);
        _fatal_error();
        return false;
    }

    // -- GET request -- //

    sprintf(http_request, "GET /%s HTTP/1.1\r\nHost: %s:%s \r\n\r\n",
            FW_FILENAME, FW_SERVER_IP, FW_SERVER_PORT);

    if (write(socket_id, http_request, strlen(http_request) + 1) < 0) {
        ESP_LOGE(TAG, "Failed to write http GET request");
        _fatal_error();
        return false;
    }

    http_state = HTTP_STATE_STATUS;

    ESP_LOGD(TAG, "-- Getting data ----");

    int         data_len     = 0;
    const char* data         = NULL;
    int         body_len     = 0;
    int         received_len = 0;

    do {
        received_len = recv(socket_id, text, TEXT_BUFFSIZE, 0);
        if (received_len < 0) {
            ESP_LOGD(TAG, "Error receiving data: errno = %d", errno);
            _fatal_error();
            return false;
        }
        data_len = received_len;
        data     = text;
        ESP_LOGD(TAG, "GET: received %d data", data_len);

        // FIXME: Handle the cases where HTTP STATUS/HEADERS would overlap
        // several data buffers...

        if (http_state == HTTP_STATE_STATUS) {
            size_t l = consume_http_status(text, data_len);
            data_len -= l;
            data += l;
            ESP_LOGD(TAG, "Data: Consumed %d, remaining %d", l, data_len);
        }
        if (http_state == HTTP_STATE_HEADERS && data_len > 0) {
            size_t l = consume_http_headers(data, data_len);
            data_len -= l;
            data += l;
            ESP_LOGD(TAG, "Data: Consumed %d, remaining %d", l, data_len);
        }
        if (http_state == HTTP_STATE_BODY && data_len > 0) {
            body_len += data_len;
            _log_buffer(data, data_len);
            err = esp_ota_write(update_handle, data, data_len);
            if (err != ESP_OK) {
                ESP_LOGE(TAG,
                         "Error while writing firmware chunk: err = 0x%04X",
                         err);
            }
        }

    } while (received_len > 0);
    ESP_LOGD(TAG, "Data received: %d", body_len);

    err = esp_ota_end(update_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "OTA end failed with error: 0x%04x", err);
        return false;
    }

    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "set boot partition failed with error: 0x%04x", err);
        return false;
    }

    ESP_LOGI(TAG, "rebooting on new firmware...");
    esp_restart();
    return true; // should never be called
}

void k_ota_start(void)
{
    connect_fw_server();
    get_firmware();
}