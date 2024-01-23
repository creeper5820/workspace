#pragma once

#include "main.h"

namespace base
{
    class Delay
    {
    private:
        uint8_t factor_us_;
        uint16_t factor_ms_;

    public:
        Delay()
        {
        }

        void us(uint16_t delay)
        {
        }

        void ms(uint32_t delay)
        {
            HAL_Delay(delay - 1);
        }
    };
}

inline base::Delay delay = base::Delay();