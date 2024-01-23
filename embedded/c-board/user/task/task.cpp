#include "main.h"
#include "base-gpio.hh"
#include "base-pwm.hh"

extern "C" {
void application();
}

void application()
{
    base::GPIO led_red(LED_RED_GPIO_Port, LED_RED_Pin);
    base::GPIO led_blue(LED_BLUE_GPIO_Port, LED_BLUE_Pin);
    base::GPIO led_green(LED_GREEN_GPIO_Port, LED_GREEN_Pin);

    while (1) {
    }
}