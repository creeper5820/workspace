#pragma once

#include "main.h"

#define DELAY             0x80

#define ST7735_MADCTL_MY  0x80
#define ST7735_MADCTL_MX  0x40
#define ST7735_MADCTL_MV  0x20
#define ST7735_MADCTL_ML  0x10
#define ST7735_MADCTL_RGB 0x00
#define ST7735_MADCTL_BGR 0x08
#define ST7735_MADCTL_MH  0x04

// AliExpress/eBay 1.8" display, default orientation

#define ST7735_IS_160X128 1
#define ST7735_WIDTH      128
#define ST7735_HEIGHT     160
#define ST7735_XSTART     0
#define ST7735_YSTART     0
#define ST7735_ROTATION   (ST7735_MADCTL_MX | ST7735_MADCTL_MY)

#define ST7735_NOP        0x00
#define ST7735_SWRESET    0x01
#define ST7735_RDDID      0x04
#define ST7735_RDDST      0x09

#define ST7735_SLPIN      0x10
#define ST7735_SLPOUT     0x11
#define ST7735_PTLON      0x12
#define ST7735_NORON      0x13

#define ST7735_INVOFF     0x20
#define ST7735_INVON      0x21
#define ST7735_DISPOFF    0x28
#define ST7735_DISPON     0x29
#define ST7735_CASET      0x2A
#define ST7735_RASET      0x2B
#define ST7735_RAMWR      0x2C
#define ST7735_RAMRD      0x2E

#define ST7735_PTLAR      0x30
#define ST7735_COLMOD     0x3A
#define ST7735_MADCTL     0x36

#define ST7735_FRMCTR1    0xB1
#define ST7735_FRMCTR2    0xB2
#define ST7735_FRMCTR3    0xB3
#define ST7735_INVCTR     0xB4
#define ST7735_DISSET5    0xB6

#define ST7735_PWCTR1     0xC0
#define ST7735_PWCTR2     0xC1
#define ST7735_PWCTR3     0xC2
#define ST7735_PWCTR4     0xC3
#define ST7735_PWCTR5     0xC4
#define ST7735_VMCTR1     0xC5

#define ST7735_RDID1      0xDA
#define ST7735_RDID2      0xDB
#define ST7735_RDID3      0xDC
#define ST7735_RDID4      0xDD

#define ST7735_PWCTR6     0xFC

#define ST7735_GMCTRP1    0xE0
#define ST7735_GMCTRN1    0xE1

// Color definitions
#define ST7735_BLACK      0x0000
#define ST7735_BLUE       0x001F
#define ST7735_RED        0xF800
#define ST7735_GREEN      0x07E0
#define ST7735_CYAN       0x07FF
#define ST7735_MAGENTA    0xF81F
#define ST7735_YELLOW     0xFFE0
#define ST7735_WHITE      0xFFFF
#define ST7735_ORANGE     0xFD60
#define ST7735_LIGHTGREEN 0x07EF
#define ST7735_COLOR565(r, g, b) \
    (((r & 0xF8) << 8) |\ ((g & 0xFC) << 3) | ((b & 0xF8) >> 3))

#define BGCOLOR    ST7735_BLACK
#define AXISCOLOR  ST7735_ORANGE
#define LIGHTCOLOR ST7735_LIGHTGREEN

// based on Adafruit ST7735 library for Arduino
inline const uint8_t
    init_cmds1[] =
        {
            // Init for 7735R, part 1 (red or green tab)
            // 15 commands in list:
            15,

            //  1: Software reset, 0 args, w/delay
            ST7735_SWRESET, DELAY,
            // 150 delay_ms delay
            150,

            //  2: Out of sleep mode, 0 args, w/delay
            ST7735_SLPOUT, DELAY,
            // 500 delay_ms delay
            255,

            //  3: Frame rate ctrl - normal mode, 3 args:
            ST7735_FRMCTR1, 3,
            // Rate = fosc/(1x2+40) * (LINE+2C+2D)
            0x01, 0x2C, 0x2D,

            //  4: Frame rate control - idle mode, 3 args:
            ST7735_FRMCTR2, 3,
            // Rate = fosc/(1x2+40) * (LINE+2C+2D)
            0x01, 0x2C, 0x2D,

            //  5: Frame rate ctrl - partial mode, 6 args:
            ST7735_FRMCTR3, 6,
            // Dot inversion mode
            0x01, 0x2C, 0x2D,
            // Line inversion mode
            0x01, 0x2C, 0x2D,

            //  6: Display inversion ctrl, 1 arg, no delay:
            ST7735_INVCTR, 1,
            // No inversion
            0x07,

            //  7: Power control, 3 args, no delay:
            ST7735_PWCTR1, 3,
            0xA2,
            //-4.6V
            0x02,
            // AUTO mode
            0x84,

            //  8: Power control, 1 arg, no delay:
            ST7735_PWCTR2, 1,
            // VGH25 = 2.4C VGSEL = -10 VGH = 3 * AVDD
            0xC5,

            //  9: Power control, 2 args, no delay:
            ST7735_PWCTR3, 2,
            // Opamp current small
            0x0A,
            // Boost frequency
            0x00,

            // 10: Power control, 2 args, no delay:
            ST7735_PWCTR4, 2,
            // BCLK/2, Opamp current small & Medium low
            0x8A,
            0x2A,

            // 11: Power control, 2 args, no delay:
            ST7735_PWCTR5, 2,
            0x8A, 0xEE,

            // 12: Power control, 1 arg, no delay:
            ST7735_VMCTR1, 1,
            0x0E,

            // 13: Don't invert display, no args, no delay
            ST7735_INVOFF, 0,

            // 14: Memory access control (directions), 1 arg:
            ST7735_MADCTL, 1,
            // row addr/col addr, bottom to top refresh
            ST7735_ROTATION,

            // 15: set color mode, 1 arg, no delay:
            ST7735_COLMOD, 1,
            // 16-bit color
            0x05},

#if (defined(ST7735_IS_128X128) || defined(ST7735_IS_160X128))
    // Init for 7735R, part 2 (1.44" display)
    init_cmds2[] =
        {
            // 2 commands in list:
            2,

            // 1: Column addr set, 4 args, no delay:
            ST7735_CASET, 4,
            // XSTART = 0
            0x00, 0x00,
            // XEND = 127
            0x00, 0x7F,

            //  2: Row addr set, 4 args, no delay:
            ST7735_RASET, 4,
            // XSTART = 0
            0x00, 0x00,
            // XEND = 127
            0x00, 0x7F},

#endif // ST7735_IS_128X128

#ifdef ST7735_IS_160X80
    init_cmds2[] =
        {                     // Init for 7735S, part 2 (160x80 display)
            3,                //  3 commands in list:
            ST7735_CASET, 4,  //  1: Column addr set, 4 args, no delay:
            0x00, 0x00,       //     XSTART = 0
            0x00, 0x4F,       //     XEND = 79
            ST7735_RASET, 4,  //  2: Row addr set, 4 args, no delay:
            0x00, 0x00,       //     XSTART = 0
            0x00, 0x9F,       //     XEND = 159
            ST7735_INVON, 0}, //  3: Invert colors
#endif

    // Init for 7735R, part 3 (red or green tab)
    init_cmds3[] =
        {

            // 4 commands in list:
            4,

            // 1: Gamma Adjustments (pos. polarity), 16 args, no delay:
            ST7735_GMCTRP1, 16,
            0x02, 0x1c, 0x07, 0x12, 0x37,
            0x32, 0x29, 0x2d, 0x29, 0x25,
            0x2B, 0x39, 0x00, 0x01, 0x03,
            0x10,

            // 2: Gamma Adjustments (neg. polarity), 16 args, no delay:
            ST7735_GMCTRN1, 16,
            0x03, 0x1d, 0x07, 0x06, 0x2E,
            0x2C, 0x29, 0x2D, 0x2E, 0x2E,
            0x37, 0x3F, 0x00, 0x00, 0x02,
            0x10,

            // 3: Normal display on, no args, w/delay
            ST7735_NORON, DELAY,
            10, //  10 delay_ms delay

            // 4: Main screen turn on, no args w/delay
            ST7735_DISPON, DELAY,
            100};