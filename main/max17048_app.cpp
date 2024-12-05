#include "max17048_app.h"

static const char *TAG = "MAX17048";

/**
 * @brief Initialize the I2C communication for the MAX17048 sensor.
 *
 * @param i2c_num The I2C port number.
 * @param i2c_addr The I2C address of the MAX17048 sensor.
 * @return ESP_OK on success, an error code otherwise.
 */
static esp_err_t max17048_init_i2c(i2c_port_t i2c_num, uint8_t i2c_addr)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = 21,               /*!< SDA pin number */
        .scl_io_num = 22,               /*!< SCL pin number */
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = 100000,
        },
    };
    esp_err_t err = i2c_param_config(i2c_num, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C config failed, err: %d", err);
        return err;
    }

    err = i2c_driver_install(i2c_num, I2C_MODE_MASTER, 0, 0, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C driver install failed, err: %d", err);
        return err;
    }

    return ESP_OK;
}

/**
 * @brief Read a 2-byte data from the MAX17048 register.
 *
 * @param i2c_num I2C port number.
 * @param i2c_addr I2C address of the MAX17048.
 * @param reg Register address to read from.
 * @param data Pointer to store the 2-byte data.
 * @return ESP_OK on success, an error code otherwise.
 */
static esp_err_t max17048_read_register(i2c_port_t i2c_num, uint8_t i2c_addr, uint8_t reg, uint16_t *data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg, true);
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (i2c_addr << 1) | I2C_MASTER_READ, true);
    i2c_master_read(cmd, (uint8_t*)data, 2, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);

    esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);

    return err;
}

/**
 * @brief Initialize the MAX17048 sensor.
 *
 * @param config Pointer to configuration structure.
 * @return ESP_OK on success, an error code otherwise.
 */
esp_err_t max17048_init(const max17048_config_t* config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // Initialize I2C communication with the provided parameters
    return max17048_init_i2c(config->i2c_num, config->i2c_addr);
}

/**
 * @brief Read the battery percentage (State of Charge) from the MAX17048.
 *
 * @param soc Pointer to the variable to store the state of charge (0-100%).
 * @return ESP_OK on success, an error code otherwise.
 */
esp_err_t max17048_get_soc(float* soc)
{
    if (soc == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t soc_raw;
    esp_err_t err = max17048_read_register(I2C_NUM_0, MAX17048_DEFAULT_ADDR, MAX17048_REG_SOC, &soc_raw);
    if (err != ESP_OK) {
        return err;
    }

    // MAX17048 state of charge is a 12-bit value, convert to percentage
    *soc = (float)(soc_raw >> 4) * 0.1f;  // Convert to percentage

    return ESP_OK;
}

/**
 * @brief Read the battery voltage from the MAX17048.
 *
 * @param voltage Pointer to the variable to store the battery voltage in volts.
 * @return ESP_OK on success, an error code otherwise.
 */
esp_err_t max17048_get_voltage(float* voltage)
{
    if (voltage == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint16_t voltage_raw;
    esp_err_t err = max17048_read_register(I2C_NUM_0, MAX17048_DEFAULT_ADDR, MAX17048_REG_VCELL, &voltage_raw);
    if (err != ESP_OK) {
        return err;
    }

    // The voltage is in 1/100th of a volt, so we divide by 100 to get volts
    *voltage = (float)voltage_raw * 0.001f;

    return ESP_OK;
}
