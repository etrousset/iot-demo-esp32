#ifndef __SI7021_H
#define __SI7021_H

#include "driver/i2c.h"

typedef enum {
    Si7013 = 0x0D,
    Si7020 = 0x14,
    Si7021 = 0x15,
} sl_sensor_id_t;

/**
 * @brief si7021_init
 * @param i2c_port
 * @param sda_gpio
 * @param sdc_gpio
 * @return
 */
esp_err_t si7021_init(i2c_port_t i2c_port, gpio_num_t sda_gpio, gpio_num_t sdc_gpio);

/**
 * @brief si7021_check_id
 *
 * Logs the ID of the sensor if any... see datasheet for more info: https://cdn.sparkfun.com/datasheets/Sensors/Weather/Si7021.pdf
 */
sl_sensor_id_t si7021_check_id();

/**
 * @brief si7021_measure_relative_humidity
 *
 * Measures the relative humidity.
 * @return the relative himidity in %, can get out of range [0.0:100.0] due to the sensor way of calculating it.
 */
float si7021_measure_relative_humidity();

/**
 * @brief si7021_measure_temperature
 *
 * Measure the current temperature. If you are also measuring relative
 * humidity, it is better to use si7021_read_temperature() that just return
 * the temperature measured during relative humidity measurement.
 * @return the temperature in °C
 */
float si7021_measure_temperature();

/**
 * @brief si7021_read_temperature
 *
 * Returns the temperature measured during last relative humidity measure.
 * @return the temperature un °C
 */
float si7021_read_temperature();

#endif // __SI7021_H
