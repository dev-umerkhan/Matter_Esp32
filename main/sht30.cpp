/*
   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <esp_err.h>
#include <esp_log.h>
#include <esp_timer.h>
#include <esp_random.h>
#include <driver/i2c.h>

#include <lib/support/CodeUtils.h>

#include "sht30.h"
#include "max17048_app.h"

static const char * TAG = "sht30";

#define I2C_MASTER_SCL_IO 21
#define I2C_MASTER_SDA_IO 22
#define I2C_MASTER_NUM I2C_NUM_0    /*!< I2C port number for master dev */
#define I2C_MASTER_FREQ_HZ 100000   /*!< I2C master clock frequency */

#define SHT30_SENSOR_ADDR 0x44      /*!< I2C address of SHT30 sensor */

typedef struct {
    sensor_config_t *config;
    esp_timer_handle_t timer;
    bool is_initialized = false;
} sensor_ctx_t;

static sensor_ctx_t s_ctx;

esp_err_t sht30_init_i2c()
{
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master = {
            .clk_speed = I2C_MASTER_FREQ_HZ,
        },
    };

    esp_err_t err = i2c_param_config(I2C_MASTER_NUM, &i2c_conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C driver, err:%d", err);
        return err;
    }

    return i2c_driver_install(I2C_MASTER_NUM, I2C_MODE_MASTER, 0, 0, 0);
}

esp_err_t sht30_read(uint8_t *data, size_t size)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT30_SENSOR_ADDR << 1) | I2C_MASTER_WRITE, true /* enable_ack */);
    // Read temperature first then humidity, with clock stretching enabled
    i2c_master_write_byte(cmd, 0x7C, true /* enable_ack */);
    i2c_master_write_byte(cmd, 0xA2, true /* enable_ack */);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    cmd = NULL;

    // Wait for measurement to complete
    vTaskDelay(pdMS_TO_TICKS(15));

    cmd = i2c_cmd_link_create();
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, (SHT30_SENSOR_ADDR << 1) | I2C_MASTER_READ, true /* enable_ack */);
    i2c_master_read(cmd, data, size, I2C_MASTER_LAST_NACK);
    i2c_master_stop(cmd);
    i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(1000));
    i2c_cmd_link_delete(cmd);
    cmd = NULL;

    return ESP_OK;
}

// Temperature in degree Celsius
float sht30_get_temp(uint16_t raw_temp)
{
    return 175.0f * (static_cast<float>(raw_temp) / 65535.0f) - 45.0f;
}

// Humidity in percentage
float sht30_get_humidity(uint16_t raw_humidity)
{
    return 100.0f * (static_cast<float>(raw_humidity) / 65535.0f);
}

esp_err_t read_sensor_data(int & temp, int & humidity, int & b_level)
{
    // foreach temperature and humidity: two bytes data, one byte for checksum
    uint8_t data[6] = {0};

    esp_err_t err = sht30_read(data, sizeof(data));
    if (err != ESP_OK) {
        return err;
    }

    uint16_t raw_temp = (data[0] << 8) | data[1];
    uint16_t raw_humidity = (data[3] << 8) | data[4];

    temp = sht30_get_temp(raw_temp);
    humidity = sht30_get_humidity(raw_humidity);
    max17048_get_soc(&b_level);

    return ESP_OK;
}

void timer_cb_internal(void *arg)
{
    auto *ctx = (sensor_ctx_t *) arg;
    if (!(ctx && ctx->config)) {
        return;
    }

    int temp, humidity, b_level;
    esp_err_t err = read_sensor_data(temp, humidity,b_level);
    if (err != ESP_OK) {
        return;
    }
    if (ctx->config->temperature.cb) {
        ctx->config->temperature.cb(ctx->config->temperature.endpoint_id, temp, ctx->config->user_data);
    }
    if (ctx->config->humidity.cb) {
        ctx->config->humidity.cb(ctx->config->humidity.endpoint_id, humidity, ctx->config->user_data);
    }
    if (ctx->config->bat_level.cb) {
        ctx->config->bat_level.cb(ctx->config->bat_level.endpoint_id, b_level, ctx->config->user_data);
    }
}

esp_err_t sensor_init(sensor_config_t *config)
{
    if (config == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    // we need at least one callback so that we can start notifying application layer
    if (config->temperature.cb == NULL || config->humidity.cb == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_ctx.is_initialized) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t err = sht30_init_i2c();
    if (err != ESP_OK) {
        return err;
    }

    // keep the pointer to config
    s_ctx.config = config;

    esp_timer_create_args_t args = {
        .callback = timer_cb_internal,
        .arg = &s_ctx,
    };

    err = esp_timer_create(&args, &s_ctx.timer);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_create failed, err:%d", err);
        return err;
    }

    err = esp_timer_start_periodic(s_ctx.timer, config->interval_ms * 1000);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_timer_start_periodic failed: %d", err);
        return err;
    }

    s_ctx.is_initialized = true;
    ESP_LOGI(TAG, "sht30 initialized successfully");

    return ESP_OK;
}