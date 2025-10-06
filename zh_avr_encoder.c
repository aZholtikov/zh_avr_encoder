#include "zh_avr_encoder.h"

#define ENCODER_DIRECTION_CW 0x10
#define ENCODER_DIRECTION_CCW 0x20

static const uint8_t _encoder_matrix[7][4] PROGMEM = {
    {0x03, 0x02, 0x01, 0x00},
    {0x23, 0x00, 0x01, 0x00},
    {0x13, 0x02, 0x00, 0x00},
    {0x03, 0x05, 0x04, 0x00},
    {0x03, 0x03, 0x04, 0x00},
    {0x03, 0x05, 0x03, 0x00},
};

TaskHandle_t zh_avr_encoder = NULL;
static QueueHandle_t _queue_handle = NULL;
static bool _is_initialized = false;

static avr_err_t _zh_avr_encoder_validate_config(const zh_avr_encoder_init_config_t *config);
static avr_err_t _zh_avr_encoder_configure_interrupts(const zh_avr_encoder_init_config_t *config, zh_avr_encoder_handle_t *handle);
static void _zh_avr_encoder_isr_processing_task(void *pvParameter);

avr_err_t zh_avr_encoder_init(const zh_avr_encoder_init_config_t *config, zh_avr_encoder_handle_t *handle)
{
    avr_err_t err = _zh_avr_encoder_validate_config(config);
    ZH_ERROR_CHECK(err == AVR_OK, err);
    handle->encoder_number = config->encoder_number;
    handle->encoder_min_value = config->encoder_min_value;
    handle->encoder_max_value = config->encoder_max_value;
    handle->encoder_step = config->encoder_step;
    handle->encoder_position = (handle->encoder_min_value + handle->encoder_max_value) / 2;
    handle->gpio_port = config->gpio_port;
    handle->a_gpio_number = config->a_gpio_number;
    handle->b_gpio_number = config->b_gpio_number;
    err = _zh_avr_encoder_configure_interrupts(config, handle);
    ZH_ERROR_CHECK(err == AVR_OK, err);
    handle->is_initialized = true;
    _is_initialized = true;
    return AVR_OK;
}

avr_err_t zh_avr_encoder_set(zh_avr_encoder_handle_t *handle, double position)
{
    ZH_ERROR_CHECK(handle->is_initialized == true, AVR_FAIL);
    ZH_ERROR_CHECK(position <= handle->encoder_max_value && position >= handle->encoder_min_value, AVR_ERR_INVALID_ARG);
    handle->encoder_position = position;
    return AVR_OK;
}

avr_err_t zh_avr_encoder_get(const zh_avr_encoder_handle_t *handle, double *position)
{
    ZH_ERROR_CHECK(handle->is_initialized == true, AVR_FAIL);
    *position = handle->encoder_position;
    return AVR_OK;
}

avr_err_t zh_avr_encoder_reset(zh_avr_encoder_handle_t *handle)
{
    ZH_ERROR_CHECK(handle->is_initialized == true, AVR_FAIL);
    handle->encoder_position = (handle->encoder_min_value + handle->encoder_max_value) / 2;
    return AVR_OK;
}

static avr_err_t _zh_avr_encoder_validate_config(const zh_avr_encoder_init_config_t *config)
{
    ZH_ERROR_CHECK(config != NULL, AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK(config->task_priority > tskIDLE_PRIORITY && config->stack_size >= configMINIMAL_STACK_SIZE, AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK(config->queue_size > 0, AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK(config->encoder_max_value > config->encoder_min_value, AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK(config->encoder_step > 0, AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK(config->gpio_port >= AVR_PORTB && config->gpio_port <= AVR_PORTD, AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK(config->a_gpio_number >= 0 && config->a_gpio_number <= 7, AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK(config->b_gpio_number >= 0 && config->b_gpio_number <= 7, AVR_ERR_INVALID_ARG);
    ZH_ERROR_CHECK(config->a_gpio_number != config->b_gpio_number, AVR_ERR_INVALID_ARG);
    return AVR_OK;
}

static avr_err_t _zh_avr_encoder_configure_interrupts(const zh_avr_encoder_init_config_t *config, zh_avr_encoder_handle_t *handle)
{
    switch (config->gpio_port)
    {
    case AVR_PORTB:
        DDRB &= ~((1 << config->a_gpio_number) | (1 << config->b_gpio_number));
        if (config->pullup == true)
        {
            PORTB |= ((1 << config->a_gpio_number) | (1 << config->b_gpio_number));
        }
        PCICR |= (1 << PCIE0);
        PCMSK0 |= ((1 << config->a_gpio_number) | (1 << config->b_gpio_number));
        break;
    case AVR_PORTC:
        DDRC &= ~((1 << config->a_gpio_number) | (1 << config->b_gpio_number));
        if (config->pullup == true)
        {
            PORTC |= ((1 << config->a_gpio_number) | (1 << config->b_gpio_number));
        }
        PCICR |= (1 << PCIE1);
        PCMSK1 |= ((1 << config->a_gpio_number) | (1 << config->b_gpio_number));
        break;
    case AVR_PORTD:
        DDRD &= ~((1 << config->a_gpio_number) | (1 << config->b_gpio_number));
        if (config->pullup == true)
        {
            PORTD |= ((1 << config->a_gpio_number) | (1 << config->b_gpio_number));
        }
        PCICR |= (1 << PCIE2);
        PCMSK2 |= ((1 << config->a_gpio_number) | (1 << config->b_gpio_number));
        break;
    default:
        return AVR_ERR_INVALID_ARG;
        break;
    }
    if (_is_initialized == false)
    {
        _queue_handle = xQueueCreate(config->queue_size, sizeof(zh_avr_encoder_handle_t));
        ZH_ERROR_CHECK(_queue_handle != NULL, AVR_ERR_NO_MEM);
        BaseType_t x_err = xTaskCreate(_zh_avr_encoder_isr_processing_task, "zh_avr_encoder", config->stack_size, NULL, config->task_priority, &zh_avr_encoder);
        if (x_err != pdPASS)
        {
            vQueueDelete(_queue_handle);
            return AVR_FAIL;
        }
    }
    return AVR_OK;
}

BaseType_t zh_avr_encoder_isr_handler(zh_avr_encoder_handle_t *handle)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    uint8_t temp = 0;
    switch (handle->gpio_port)
    {
    case AVR_PORTB:
        temp = pgm_read_byte(&_encoder_matrix[handle->encoder_state & 0x0F][(((PINB & (1 << handle->b_gpio_number)) == 0 ? 0 : 1) << 1) | ((PINB & (1 << handle->a_gpio_number)) == 0 ? 0 : 1)]);
        break;
    case AVR_PORTC:
        temp = pgm_read_byte(&_encoder_matrix[handle->encoder_state & 0x0F][(((PINC & (1 << handle->b_gpio_number)) == 0 ? 0 : 1) << 1) | ((PINC & (1 << handle->a_gpio_number)) == 0 ? 0 : 1)]);
        break;
    case AVR_PORTD:
        temp = pgm_read_byte(&_encoder_matrix[handle->encoder_state & 0x0F][(((PIND & (1 << handle->b_gpio_number)) == 0 ? 0 : 1) << 1) | ((PIND & (1 << handle->a_gpio_number)) == 0 ? 0 : 1)]);
        break;
    default:
        break;
    }
    if (temp != handle->encoder_state)
    {
        handle->encoder_state = temp;
        switch (handle->encoder_state & 0x30)
        {
        case ENCODER_DIRECTION_CW:
            if (handle->encoder_position < handle->encoder_max_value)
            {
                handle->encoder_position = handle->encoder_position + handle->encoder_step;
                if (handle->encoder_position > handle->encoder_max_value)
                {
                    handle->encoder_position = handle->encoder_max_value;
                }
                xQueueSendFromISR(_queue_handle, handle, &xHigherPriorityTaskWoken);
            }
            break;
        case ENCODER_DIRECTION_CCW:
            if (handle->encoder_position > handle->encoder_min_value)
            {
                handle->encoder_position = handle->encoder_position - handle->encoder_step;
                if (handle->encoder_position < handle->encoder_min_value)
                {
                    handle->encoder_position = handle->encoder_min_value;
                }
                xQueueSendFromISR(_queue_handle, handle, &xHigherPriorityTaskWoken);
            }
            break;
        default:
            break;
        }
    }
    return xHigherPriorityTaskWoken;
}

static void _zh_avr_encoder_isr_processing_task(void *pvParameter)
{
    zh_avr_encoder_handle_t queue = {0};
    zh_avr_encoder_event_on_isr_t event = {0};
    while (xQueueReceive(_queue_handle, &queue, portMAX_DELAY) == pdTRUE)
    {
        event.encoder_number = queue.encoder_number;
        event.encoder_position = queue.encoder_position;
        extern void zh_avr_encoder_event_handler(zh_avr_encoder_event_on_isr_t * event);
        zh_avr_encoder_event_handler(&event);
    }
    vTaskDelete(NULL);
}