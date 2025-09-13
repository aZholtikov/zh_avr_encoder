# FreeRTOS based AVR library for rotary encoder

## Features

1. Support some encoders on one device.

## Using

In an existing project, run the following command to install the components:

```text
cd ../your_project/lib
git clone http://git.zh.com.ru/avr_libraries/zh_avr_free_rtos
git clone http://git.zh.com.ru/avr_libraries/zh_avr_common
git clone http://git.zh.com.ru/avr_libraries/zh_avr_encoder
```

In the application, add the component:

```c
#include "zh_avr_encoder.h"
```

## Examples

One encoder on device:

```c
#include "avr/io.h"
#include "stdio.h"
#include "zh_avr_encoder.h"

#define BAUD_RATE 9600
#define BAUD_PRESCALE (F_CPU / 16 / BAUD_RATE - 1)

int usart(char byte, FILE *stream)
{
    while ((UCSR0A & (1 << UDRE0)) == 0)
    {
    }
    UDR0 = byte;
    return 0;
}
FILE uart = FDEV_SETUP_STREAM(usart, NULL, _FDEV_SETUP_WRITE);

zh_avr_encoder_handle_t encoder_handle = {0};

int main(void)
{
    UBRR0H = (BAUD_PRESCALE >> 8);
    UBRR0L = BAUD_PRESCALE;
    UCSR0B = (1 << RXEN0) | (1 << TXEN0);
    UCSR0C = (1 << UCSZ01) | (1 << UCSZ00);
    stdout = &uart;
    zh_avr_encoder_init_config_t encoder_init_config = ZH_AVR_ENCODER_INIT_CONFIG_DEFAULT();
    encoder_init_config.gpio_port = AVR_PORTD;
    encoder_init_config.a_gpio_number = PORTD5;
    encoder_init_config.b_gpio_number = PORTD6;
    encoder_init_config.pullup = true;
    encoder_init_config.encoder_min_value = -10;
    encoder_init_config.encoder_max_value = 20;
    encoder_init_config.encoder_step = 1;
    encoder_init_config.encoder_number = 1;
    zh_avr_encoder_init(&encoder_init_config, &encoder_handle);
    double position = 0;
    zh_avr_encoder_get(&encoder_handle, &position);
    printf("Encoder position %d.\n", (int)position);
    zh_avr_encoder_set(&encoder_handle, 5);
    zh_avr_encoder_reset(&encoder_handle);
    vTaskStartScheduler();
    return 0;
}

void zh_avr_encoder_event_handler(zh_avr_encoder_event_on_isr_t *event) // Do not delete!
{
    printf("Encoder number %d position %d.\n", event->encoder_number, (int)event->encoder_position);
    printf("Interrupt Task Remaining Stack Size %d.\n", uxTaskGetStackHighWaterMark(NULL));
}

// ISR(PCINT0_vect) // For AVR_PORTB.
// ISR(PCINT1_vect) // For AVR_PORTC.
ISR(PCINT2_vect) // For AVR_PORTD.
{
    if (zh_avr_encoder_isr_handler(&encoder_handle) == pdTRUE)
    {
        portYIELD();
    }
}
```
