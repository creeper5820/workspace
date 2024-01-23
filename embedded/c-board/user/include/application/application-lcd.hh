#pragma once
#include "spi.h"
#include "gpio.h"

#include "base-gpio.hh"
#include "base-time.hh"
#include "base-pwm.hh"
#include "base-spi.hh"

#include "application-lcd-def.hh"

namespace application
{

    class LCD
    {
    public:
        LCD(base::GPIO reset,
            base::GPIO dc,
            base::GPIO cs,
            base::PWM pwm,
            base::SPI spi)

            : reset_(reset),
              dc_(dc),
              cs_(cs),
              pwm_(pwm),
              spi_(spi)
        {
            init();
        }

        LCD(GPIO_TypeDef *reset_type, uint16_t reset_pin,
            GPIO_TypeDef *dc_type, uint16_t dc_pin,
            GPIO_TypeDef *cs_type, uint16_t cs_pin,
            TIM_HandleTypeDef *htim, uint32_t channel,
            SPI_HandleTypeDef *spi)

            : LCD(base::GPIO(reset_type, reset_pin),
                  base::GPIO(dc_type, dc_pin),
                  base::GPIO(cs_type, cs_pin),
                  base::PWM(htim, channel),
                  base::SPI(spi))
        {
        }

        void init()
        {
            cs_.reset();
            reset_.reset();
            delay.ms(5);
            reset_.set();

            command_lists(init_cmds1);
            command_lists(init_cmds2);
            command_lists(init_cmds3);

            cs_.set();
        }

        void set_transmit_type(base::SpiTransmitType type)
        {
            spi_transmit_type_ = type;
        }

        void draw_point(uint16_t x, uint16_t y, uint16_t color)
        {
            if (x > ST7735_WIDTH || x < 0 || y > ST7735_HEIGHT || y < 0)
                return;

            cs_.reset();

            set_address_window(x, y, x + 1, y + 1);
            uint8_t send[] = {
                (uint8_t)(color >> 8),
                (uint8_t)(color & 0XFF)};
            data(send, sizeof(send));

            cs_.set();
        }

        void fill_color(uint16_t x, uint16_t y, uint16_t w, uint16_t h, uint16_t color)
        {
            // clipping
            if ((x >= ST7735_WIDTH) || (y >= ST7735_HEIGHT)) return;
            if ((x + w - 1) >= ST7735_WIDTH) w = ST7735_WIDTH - x;
            if ((y + h - 1) >= ST7735_HEIGHT) h = ST7735_HEIGHT - y;

            cs_.reset();
            set_address_window(x, y, x + w - 1, y + h - 1);

            uint8_t send[] = {(uint8_t)(color >> 8), (uint8_t)(color & 0xFF)};

            dc_.set();

            for (y = h; y > 0; y--) {
                for (x = w; x > 0; x--) {
                    spi_.transmit(
                        base::SPI_TRANSMIT_DMA_TYPE,
                        send, sizeof(send),
                        HAL_MAX_DELAY);
                }
            }

            cs_.set();
        }

    private:
        // Reset pin for lcd
        base::GPIO reset_;
        // the pin to choose sending data or command
        base::GPIO dc_;
        // the pin to choose the lcd
        base::GPIO cs_;
        // pwmgenertor to control the light of lcd
        base::PWM pwm_;
        // spi controller to send data
        base::SPI spi_;
        // spi transmit and receive type
        base::SpiTransmitType spi_transmit_type_ = base::SPI_TRANSMIT_TYPE;

    private:
        void command(const uint8_t &send)
        {
            cs_.reset();
            dc_.reset();

            spi_.transmit(spi_transmit_type_, (uint8_t *)&send, 1);

            cs_.set();
        }

        void data(const uint8_t *send, size_t size)
        {
            cs_.reset();
            dc_.set();

            spi_.transmit(spi_transmit_type_, (uint8_t *)&send, size);

            cs_.set();
        }

        void command_lists(const uint8_t *list)
        {
            uint8_t number_commands;
            uint8_t number_args;
            uint16_t delay_ms;

            number_commands = *list++;

            while (number_commands--) {
                uint8_t cmd = *list++;
                command(cmd);

                number_args = *list++;

                // If high bit set, delay follows args
                delay_ms = number_args & DELAY;
                number_args &= ~DELAY;

                if (number_args) {
                    data((uint8_t *)list, number_args);
                    list += number_args;
                }

                if (delay_ms) {
                    delay_ms = *list++;
                    if (delay_ms == 255) delay_ms = 500;
                    delay.ms(delay_ms);
                }
            }
        }

        void set_address_window(uint8_t x_1, uint8_t y_1, uint8_t x_2, uint8_t y_2)
        {
            // column address set
            command(ST7735_CASET);

            uint8_t send[] = {
                0x00,
                (uint8_t)(x_1 + ST7735_XSTART),
                0x00,
                (uint8_t)(x_2 + ST7735_XSTART)};

            data(send, sizeof(send));

            // row address set
            command(ST7735_RASET);

            send[1] = y_1 + ST7735_YSTART;
            send[3] = y_2 + ST7735_YSTART;

            data(send, sizeof(send));

            // write to RAM
            command(ST7735_RAMWR);
        }

    }; // class LCD end
} // namespace Application end
