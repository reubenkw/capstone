#ifndef GPIO_H
#define GPIO_H

#include "driver/i2c.h"
#include "led_strip.h"

// buck and boost
#define GPIO_BUCK 21
#define GPIO_BOOST 14

// led strip is only on dev board
#define BLINK_GPIO CONFIG_BLINK_GPIO

// leds on PCB, accessible from MCU_M
#define LED_1 15
#define LED_2 16
#define LED_3 17

#define GPIO_LED_PIN_SEL ( (1ULL<<LED_1) | (1ULL<<LED_2) |(1ULL<<LED_3) )

// limit switches, accessible from MCU_M (for now)
#define LIMIT_X_MIN 2
#define LIMIT_X_MAX 44
#define LIMIT_Y_MIN 47
#define LIMIT_Y_MAX 43
#define LIMIT_Z 1

#define GPIO_LIMIT_PIN_SEL  ( \
    (1ULL<<LIMIT_X_MIN) | (1ULL<<LIMIT_X_MAX) | \
    (1ULL<<LIMIT_Y_MIN) | (1ULL<<LIMIT_Y_MAX) | \
    (1ULL<<LIMIT_Z) \
)

extern uint8_t limit_pins[5];

void init_dev_led();
void dev_led_set_color(int r, int g, int b);
void initialize_led();
void init_boost();
void init_limit_gpio();

#endif // h
