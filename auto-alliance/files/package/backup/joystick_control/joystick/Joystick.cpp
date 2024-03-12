//
// Created by colin on 2023/2/4.
//

#include "Joystick.h"

#include <memory>
#include <utility>

Joystick::Joystick(std::string device, bool blocked) : dev(std::move(device)), block(blocked), xbox_fd(-1) {
    reset();
}


int Joystick::read() {
    int len = 0, type, number, value;
    struct js_event js{};
    tcflush(xbox_fd, TCIOFLUSH);
    len = ::read(xbox_fd, &js, sizeof(struct js_event));


    type = js.type;
    number = js.number;
    value = js.value;

    map.time = js.time;

    map.trigger_a = XBOX_BUTTON_OFF;
    map.trigger_b = XBOX_BUTTON_OFF;
    map.trigger_x = XBOX_BUTTON_OFF;
    map.trigger_y = XBOX_BUTTON_OFF;
    map.trigger_lb = XBOX_BUTTON_OFF;
    map.trigger_rb = XBOX_BUTTON_OFF;

    if (type == JS_EVENT_BUTTON) {
        switch (number) {
            case XBOX_BUTTON_A:
                map.trigger_a = (value == XBOX_BUTTON_OFF) && (map.a == XBOX_BUTTON_ON);
                map.a = value;
                break;

            case XBOX_BUTTON_B:
                map.trigger_b = (value == XBOX_BUTTON_OFF) && (map.b == XBOX_BUTTON_ON);
                map.b = value;
                break;

            case XBOX_BUTTON_X:
                map.trigger_x = (value == XBOX_BUTTON_OFF) && (map.x == XBOX_BUTTON_ON);
                map.x = value;
                break;
            case XBOX_BUTTON_Y:
                map.trigger_y = (value == XBOX_BUTTON_OFF) && (map.y == XBOX_BUTTON_ON);
                map.y = value;
                break;
            case XBOX_BUTTON_LB:
                map.trigger_lb = (value == XBOX_BUTTON_OFF) && (map.lb == XBOX_BUTTON_ON);
                map.lb = value;
                break;

            case XBOX_BUTTON_RB:
                map.trigger_rb = (value == XBOX_BUTTON_OFF) && (map.rb == XBOX_BUTTON_ON);
                map.rb = value;
                break;

            case XBOX_BUTTON_START:
                map.start = value;
                break;

            case XBOX_BUTTON_BACK:
                map.back = value;
                break;

            case XBOX_BUTTON_HOME:
                map.home = value;
                break;

            case XBOX_BUTTON_LO:
                map.lo = value;
                break;

            case XBOX_BUTTON_RO:
                map.ro = value;
                break;

            default:
                break;
        }
    } else if (type == JS_EVENT_AXIS) {
        switch (number) {
            case XBOX_AXIS_LX:
                map.lx = value;
                break;

            case XBOX_AXIS_LY:
                map.ly = value;
                break;

            case XBOX_AXIS_RX:
                map.rx = value;
                break;

            case XBOX_AXIS_RY:
                map.ry = value;
                break;

            case XBOX_AXIS_LT:
                map.lt = value;
                break;

            case XBOX_AXIS_RT:
                map.rt = value;
                break;

            case XBOX_AXIS_XX:
                map.xx = value;
                break;

            case XBOX_AXIS_YY:
                map.yy = value;
                break;

            default:
                break;
        }
    }

    return len;
}

Joystick::~Joystick() {

    close(xbox_fd);
}


void Joystick::reset() {
    if (xbox_fd > 0) {
        close(xbox_fd);
    }
    if (block) {
        xbox_fd = ::open(dev.c_str(), O_RDONLY);
    } else {
        xbox_fd = ::open(dev.c_str(), O_RDONLY | O_NONBLOCK);
    }

}

bool Joystick::isValid() {
    return xbox_fd > 0;
}
