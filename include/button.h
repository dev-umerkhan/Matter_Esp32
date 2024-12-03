#pragma once

#include "driver/gpio.h"
#include "button_gpio.h"
#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief gpio button configuration
 *
 */

/**
 * @brief Initialize gpio button
 *
 * @param config pointer of configuration struct
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_ARG   Arguments is NULL.
 */
esp_err_t button_gpio_init(const button_gpio_config_t *config);

/**
 * @brief Deinitialize gpio button
 *
 * @param gpio_num gpio number of button
 *
 * @return Always return ESP_OK
 */
esp_err_t button_gpio_deinit(gpio_num_t gpio_num);

/**
 * @brief Get current level on button gpio
 *
 * @param gpio_num gpio number of button, it will be treated as a uint32_t variable.
 *
 * @return Level on gpio
 */
uint8_t button_gpio_get_key_level(void *gpio_num);

/**
 * @brief Sets up interrupt for GPIO button.
 *
 * @param gpio_num gpio number of button
 * @param intr_type The type of GPIO interrupt.
 * @param isr_handler The ISR (Interrupt Service Routine) handler function.
 * @param args Arguments to be passed to the ISR handler function.
 * @return Always return ESP_OK
 */
esp_err_t button_gpio_set_intr(gpio_num_t gpio_num, gpio_int_type_t intr_type, gpio_isr_t isr_handler, void *args);

/**
 * @brief Enable or disable interrupt for GPIO button.
 *
 * @param gpio_num gpio number of button
 * @param enable enable or disable
 * @return Always return ESP_OK
 */
esp_err_t button_gpio_intr_control(gpio_num_t gpio_num, bool enable);

/**
 * @brief Enable or disable GPIO wakeup functionality.
 *
 * This function allows enabling or disabling GPIO wakeup feature.
 *
 * @param gpio_num GPIO number for wakeup functionality.
 * @param active_level Active level of the GPIO when triggered.
 * @param enable Enable or disable the GPIO wakeup.
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_INVALID_STATE if trigger was not active or in conflict.
 */
esp_err_t button_gpio_enable_gpio_wakeup(gpio_num_t gpio_num, uint8_t active_level, bool enable);

#ifdef __cplusplus
}
#endif