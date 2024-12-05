#ifndef MAX17048_H
#define MAX17048_H

#include <esp_err.h>
#include <esp_log.h>
#include <driver/i2c.h>

#define MAX17048_DEFAULT_ADDR 0x36  /*!< Default I2C address of MAX17048 sensor */

#define MAX17048_REG_VCELL 0x02     /*!< Register for battery voltage */
#define MAX17048_REG_SOC 0x04       /*!< Register for State of Charge (SOC) */
#define MAX17048_REG_MODE 0x06      /*!< Register for mode control */

/**
 * @brief Configuration structure for MAX17048 sensor
 */
typedef struct {
    i2c_port_t i2c_num;        /*!< I2C port number */
    uint8_t i2c_addr;          /*!< I2C address of the MAX17048 sensor */
} max17048_config_t;

/**
 * @brief Initialize the MAX17048 sensor.
 *
 * @param config Pointer to configuration structure.
 * @return ESP_OK on success, an error code otherwise.
 */
esp_err_t max17048_init(const max17048_config_t* config);

/**
 * @brief Read the battery percentage (State of Charge) from the MAX17048.
 *
 * @param soc Pointer to the variable to store the state of charge (0-100%).
 * @return ESP_OK on success, an error code otherwise.
 */
esp_err_t max17048_get_soc(int* soc);

/**
 * @brief Read the battery voltage from the MAX17048.
 *
 * @param voltage Pointer to the variable to store the battery voltage in volts.
 * @return ESP_OK on success, an error code otherwise.
 */
esp_err_t max17048_get_voltage(float* voltage);

#endif // MAX17048_H
