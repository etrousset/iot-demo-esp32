
#include "si7021.h"
#include "driver/i2c.h"
#include "esp_log.h"

#define TAG "SI7021"

#define SI7021_I2C_ADDR 0x40

static i2c_port_t _i2c_port = I2C_NUM_MAX;

esp_err_t si7021_init(i2c_port_t i2c_port, gpio_num_t sda_gpio, gpio_num_t sdc_gpio)
{
    ESP_LOGD(TAG, "Initialising with I2C port = %d, SDA gpio = %d, SDC GPIO = %d", i2c_port, sda_gpio, sdc_gpio);

    _i2c_port = i2c_port;

    i2c_config_t conf;
    conf.mode             = I2C_MODE_MASTER;
    conf.sda_io_num       = sda_gpio;
    conf.sda_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.scl_io_num       = sdc_gpio;
    conf.scl_pullup_en    = GPIO_PULLUP_ENABLE;
    conf.master.clk_speed = 100000;
    i2c_param_config(_i2c_port, &conf);

    i2c_driver_install(_i2c_port, I2C_MODE_MASTER, 0, 0, 0);

    sl_sensor_id_t id = si7021_check_id();
    if(id == Si7013 || id == Si7020 || id == Si7021)
        return ESP_OK
                ;
    else {
        ESP_LOGE(TAG, "Didn't detect a valid device");
        return ESP_FAIL;
    }
}

sl_sensor_id_t si7021_check_id()
{
    uint8_t          data;
    esp_err_t        err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (SI7021_I2C_ADDR << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0xFC, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0xC9, true));
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (SI7021_I2C_ADDR << 1) | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, &data, 0 /*ACK_VAL*/));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    err = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "===> I2C error: 0x%02x", err);
    i2c_cmd_link_delete(cmd);

    ESP_LOGD(TAG, "Device ID = 0x%02x", data);

    return data;
}

float si7021_measure_relative_humidity()
{
    uint8_t          buffer[2] = {};
    esp_err_t        err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (SI7021_I2C_ADDR << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0xE5, true));
    err = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "===> I2C error: 0x%02x", err);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(50 / portTICK_PERIOD_MS);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (SI7021_I2C_ADDR << 1) | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, buffer, 0 /*ACK_VAL*/));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, buffer + 1, 1 /*ACK_VAL*/));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    err = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "===> I2C error: 0x%02x", err);
    i2c_cmd_link_delete(cmd);

    uint16_t rh_code = (buffer[0] << 8 | buffer[1]);
    float    rh      = 125.f * rh_code / 65536 - 6.f;
    return rh;
}

float si7021_measure_temperature()
{
    uint8_t          buffer[2] = {};
    esp_err_t        err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (SI7021_I2C_ADDR << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0xE3, true));
    err = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "===> I2C error: 0x%02x", err);
    i2c_cmd_link_delete(cmd);

    vTaskDelay(50 / portTICK_PERIOD_MS);

    cmd = i2c_cmd_link_create();
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (SI7021_I2C_ADDR << 1) | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, buffer, 0 /*ACK_VAL*/));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, buffer + 1, 1 /*ACK_VAL*/));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    err = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "===> I2C error: 0x%02x", err);
    i2c_cmd_link_delete(cmd);

    uint16_t temp_code = (buffer[0] << 8 | buffer[1]);
    float    t_celcius = 176.72f * temp_code / 65536 - 46.85f;
    return t_celcius;
}

float si7021_read_temperature()
{
    uint8_t          buffer[2] = {};
    esp_err_t        err;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();

    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (SI7021_I2C_ADDR << 1) | I2C_MASTER_WRITE, true));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, 0xE0, true));
    ESP_ERROR_CHECK(i2c_master_start(cmd));
    ESP_ERROR_CHECK(i2c_master_write_byte(cmd, (SI7021_I2C_ADDR << 1) | I2C_MASTER_READ, true));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, buffer, 0 /*ACK_VAL*/));
    ESP_ERROR_CHECK(i2c_master_read_byte(cmd, buffer + 1, 1 /*ACK_VAL*/));
    ESP_ERROR_CHECK(i2c_master_stop(cmd));

    err = i2c_master_cmd_begin(_i2c_port, cmd, 1000 / portTICK_RATE_MS);
    if (err != ESP_OK)
        ESP_LOGE(TAG, "===> I2C error: 0x%02x", err);
    i2c_cmd_link_delete(cmd);

    uint16_t temp_code = (buffer[0] << 8 | buffer[1]);
    float    t_celcius = 176.72f * temp_code / 65536 - 46.85f;

    return t_celcius;
}
