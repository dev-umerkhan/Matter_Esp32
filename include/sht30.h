/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

// This file implements the SHT30 temperature and humidity sensor driver.
// This is implemented keeping the Matter requirements in mind.
//
// Datasheet: https://sensirion.com/media/documents/213E6A3B/63A5A569/Datasheet_SHT3x_DIS.pdf

#pragma once

#include <esp_err.h>

using sht30_sensor_cb_t = void (*)(uint16_t endpoint_id, float value, void *user_data);

typedef struct {
    struct {
        // This callback functon will be called periodically to report the temperature.
        sht30_sensor_cb_t cb = NULL;
        // endpoint_id associated with temperature sensor
        uint16_t endpoint_id;
    } temperature;

    struct {
        // This callback functon will be called periodically to report the humidity.
        sht30_sensor_cb_t cb = NULL;
        // endpoint_id associated with humidity sensor
        uint16_t endpoint_id;
    } humidity;

    // user data
    void *user_data = NULL;

    // polling interval in milliseconds, defaults to 5000 ms
    uint32_t interval_ms = 5000;
} sht30_sensor_config_t;

/**
 * @brief Initialize sensor driver. This function should be called only once
 *        When initializing, at least one callback should be provided, else it
 *        returns ESP_ERR_INVALID_ARG.
 *
 * @param config sensor configurations. This should last for the lifetime of the driver
 *               as driver layer do not make a copy of this object.
 *
 * @return esp_err_t - ESP_OK on success,
 *                     ESP_ERR_INVALID_ARG if config is NULL
 *                     ESP_ERR_INVALID_STATE if driver is already initialized
 *                     appropriate error code otherwise
 */
esp_err_t sht30_sensor_init(sht30_sensor_config_t *config);
esp_err_t sht30_get_read_temp_and_humidity(float & temp, float & humidity);
float sht30_get_humidity(uint16_t raw_humidity);
float sht30_get_temp(uint16_t raw_temp);
esp_err_t sht30_read(uint8_t *data, size_t size);
esp_err_t sht30_init_i2c();
