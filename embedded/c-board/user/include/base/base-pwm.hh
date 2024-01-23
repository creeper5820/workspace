#pragma once

#include "main.h"
#include "tim.h"

namespace base
{
    class PWM
    {
    private:
        TIM_HandleTypeDef *htim_;
        uint32_t channel_;

    public:
        /// @brief PWM生成器的构造函数
        /// @param htim 定时器的句柄
        /// @param channel 定时器的通道
        PWM(TIM_HandleTypeDef *htim, uint32_t channel)
            : htim_(htim), channel_(channel)
        {
            HAL_TIM_PWM_Start(htim_, channel_);
        }

        /// @brief 初始化
        void init()
        {
            HAL_TIM_PWM_Start(htim_, channel_);
        }

        /// @brief 设置PWM的非空值
        /// @param pwm 其为值
        void set_pwm(const uint32_t pwm)
        {
            if (pwm < 0 || pwm > htim_->Init.Period)
                return;

            __HAL_TIM_SET_COMPARE(htim_, channel_, pwm);
        }

        void limit(float &ratio, float limit)
        {
            if (ratio > limit)
                ratio = limit;

            if (ratio < -limit)
                ratio = -limit;
        }

        /// @brief 设置PWM的非空比例
        /// @param ratio 其为比例, 范围 0.0 ~ 1.0
        void set_pwm_ratio(float ratio)
        {
            limit(ratio, 1);
            __HAL_TIM_SET_COMPARE(htim_, channel_, ratio * (htim_->Init.Period));
        }
    };
}