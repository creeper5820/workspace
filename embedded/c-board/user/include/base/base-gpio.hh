#pragma once

#include "gpio.h"

namespace base
{
    class GPIO
    {
    private:
        GPIO_TypeDef *type_;
        uint16_t pin_;

    public:
        GPIO(GPIO_TypeDef *type, uint16_t pin)
            : type_(type),
              pin_(pin)
        {
        }

        void set()
        {
            HAL_GPIO_WritePin(type_, pin_, GPIO_PIN_SET);
        }

        void reset()
        {
            HAL_GPIO_WritePin(type_, pin_, GPIO_PIN_RESET);
        }

        void toggle()
        {
            HAL_GPIO_TogglePin(type_, pin_);
        }

        GPIO_PinState status()
        {
            return HAL_GPIO_ReadPin(type_, pin_);
        }
    };
}