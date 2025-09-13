#pragma once

#include "FreeRTOS.h"
#include "semphr.h"
#include "avr_err.h"
#include "avr_port.h"
#include "stdbool.h"
#include "avr/interrupt.h"
#include "avr/pgmspace.h"

#define ZH_AVR_ENCODER_INIT_CONFIG_DEFAULT()   \
    {                                          \
        .task_priority = configMAX_PRIORITIES, \
        .stack_size = 124,                     \
        .queue_size = 1,                       \
        .gpio_port = 0,                        \
        .a_gpio_number = 0,                    \
        .b_gpio_number = 0,                    \
        .pullup = false,                       \
        .encoder_min_value = -100,             \
        .encoder_max_value = 100,              \
        .encoder_step = 1,                     \
        .encoder_number = 0}

#ifdef __cplusplus
extern "C"
{
#endif

    typedef struct // Structure for initial initialization of encoder.
    {
        uint8_t task_priority;     // Task priority for the encoder isr processing. @note It is not recommended to set a value less than configMAX_PRIORITIES.
        uint16_t stack_size;       // Stack size for task for the encoder isr processing processing. @note The minimum size is 124 bytes.
        uint8_t queue_size;        // Queue size for task for the encoder processing. Depends on the number of encoders.
        uint8_t gpio_port;         // Encoder GPIO port. @note Must be same for A and B GPIO.
        uint8_t a_gpio_number;     // Encoder A GPIO number.
        uint8_t b_gpio_number;     // Encoder B GPIO number.
        bool pullup;               // Using internal pullup resistors.
        int32_t encoder_min_value; // Encoder min value. @note Must be less than encoder_max_value.
        int32_t encoder_max_value; // Encoder max value. @note Must be greater than encoder_min_value.
        double encoder_step;       // Encoder step. @note Must be greater than 0.
        uint8_t encoder_number;    // Unique encoder number.
    } zh_avr_encoder_init_config_t;

    typedef struct // Encoder handle.
    {
        uint8_t gpio_port;         // Encoder GPIO port.
        uint8_t a_gpio_number;     // Encoder A GPIO number.
        uint8_t b_gpio_number;     // Encoder B GPIO number.
        int32_t encoder_min_value; // Encoder min value.
        int32_t encoder_max_value; // Encoder max value.
        double encoder_step;       // Encoder step.
        double encoder_position;   // Encoder position.
        uint8_t encoder_number;    // Encoder unique number.
        uint8_t encoder_state;     // Encoder internal state.
        bool is_initialized;       // Encoder initialization flag.
    } zh_avr_encoder_handle_t;

    typedef struct // Structure for sending data to the event handler when cause an interrupt. @note Should be used with zh_avr_encoder event base.
    {
        uint8_t encoder_number;  // Encoder unique number.
        double encoder_position; // Encoder current position.
    } zh_avr_encoder_event_on_isr_t;

    /**
     * @brief Initialize encoder.
     *
     * @note The encoder will be set to the position (encoder_min_value + encoder_max_value)/2.
     *
     * @param[in] config Pointer to encoder initialized configuration structure. Can point to a temporary variable.
     * @param[out] handle Pointer to unique encoder handle.
     *
     * @note Before initialize the encoder recommend initialize zh_avr_encoder_init_config_t structure with default values.
     *
     * @code zh_avr_encoder_init_config_t config = ZH_AVR_ENCODER_INIT_CONFIG_DEFAULT() @endcode
     *
     * @return AVR_OK if success or an error code otherwise.
     */
    avr_err_t zh_avr_encoder_init(const zh_avr_encoder_init_config_t *config, zh_avr_encoder_handle_t *handle);

    /**
     * @brief Set encoder position.
     *
     * @param[in, out] handle Pointer to unique encoder handle.
     * @param[in] position Encoder position (must be between encoder_min_value and encoder_max_value).
     *
     * @return AVR_OK if success or an error code otherwise.
     */
    avr_err_t zh_avr_encoder_set(zh_avr_encoder_handle_t *handle, double position);

    /**
     * @brief Get encoder position.
     *
     * @param[in] handle Pointer to unique encoder handle.
     * @param[out] position Encoder position.
     *
     * @return AVR_OK if success or an error code otherwise.
     */
    avr_err_t zh_avr_encoder_get(const zh_avr_encoder_handle_t *handle, double *position);

    /**
     * @brief Reset encoder position.
     *
     * @note The encoder will be set to the position (encoder_min_value + encoder_max_value)/2.
     *
     * @param[in, out] handle Pointer to unique encoder handle.
     *
     * @return AVR_OK if success or an error code otherwise.
     */
    avr_err_t zh_avr_encoder_reset(zh_avr_encoder_handle_t *handle);

    /**
     * @brief Encoder ISR handler.
     */
    BaseType_t zh_avr_encoder_isr_handler(zh_avr_encoder_handle_t *handle);

#ifdef __cplusplus
}
#endif