//
// Created by colin on 2023/2/4.
//

#ifndef JOYSTICK_JOYSTICK_H
#define JOYSTICK_JOYSTICK_H

#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <termios.h>
#include <linux/joystick.h>

#define XBOX_TYPE_BUTTON    0x01
#define XBOX_TYPE_AXIS      0x02

#define XBOX_BUTTON_A       0x00
#define XBOX_BUTTON_B       0x01
#define XBOX_BUTTON_X       0x02
#define XBOX_BUTTON_Y       0x03
#define XBOX_BUTTON_LB      0x04
#define XBOX_BUTTON_RB      0x05
#define XBOX_BUTTON_START   0x06
#define XBOX_BUTTON_BACK    0x07
#define XBOX_BUTTON_HOME    0x08
#define XBOX_BUTTON_LO      0x09    /* 左摇杆按键 */
#define XBOX_BUTTON_RO      0x0a    /* 右摇杆按键 */

#define XBOX_BUTTON_ON      0x01
#define XBOX_BUTTON_OFF     0x00

#define XBOX_AXIS_LX        0x00    /* 左摇杆X轴 */
#define XBOX_AXIS_LY        0x01    /* 左摇杆Y轴 */
#define XBOX_AXIS_RX        0x03    /* 右摇杆X轴 */
#define XBOX_AXIS_RY        0x04    /* 右摇杆Y轴 */
#define XBOX_AXIS_LT        0x02
#define XBOX_AXIS_RT        0x05
#define XBOX_AXIS_XX        0x06    /* 方向键X轴 */
#define XBOX_AXIS_YY        0x07    /* 方向键Y轴 */

#define XBOX_AXIS_VAL_UP        (-32767)
#define XBOX_AXIS_VAL_DOWN      32767
#define XBOX_AXIS_VAL_LEFT      (-32767)
#define XBOX_AXIS_VAL_RIGHT     32767

#define XBOX_AXIS_VAL_MIN       (-32767)
#define XBOX_AXIS_VAL_MAX       32767
#define XBOX_AXIS_VAL_MID       0x00

#include <string>
#include <atomic>


class Joystick {
public:
    Joystick(std::string device, bool blocked);

    int read();

    void reset();

    bool isValid();

    ~Joystick();

    struct xbox_map {
        int time;
        int a, trigger_a;
        int b, trigger_b;
        int x, trigger_x;
        int y, trigger_y;
        int lb, trigger_lb;
        int rb, trigger_rb;
        int start;
        int back;
        int home;
        int lo;
        int ro;

        int lx;
        int ly;
        int rx;
        int ry;

        int lt;
        int rt;

        int xx;
        int yy;

    } map{0};

private:
    int xbox_fd{};
    std::string dev;
    bool block;
};


#endif //JOYSTICK_JOYSTICK_H
