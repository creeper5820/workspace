//
// Created by soulde on 2023/4/5.
//
#pragma once

//#include "Serial.h"
#include <cstdint>

class Serial {

protected:
    int _handle;

public:
    Serial() = default;

    Serial(const char *portName, uint32_t baudrate=UGAS_115200, uint8_t parity=NOPARITY, uint8_t databit=DATABIT8, uint8_t stopbit=ONESTOPBIT,
           bool synFlag = true) {
        _handle = open(portName, O_RDWR | O_NOCTTY);
        if (_handle >= 0) {
            std::cout << "Serial[" << _handle << "] " << portName << " start success" << std::endl;
        }else{
            std::cerr<< "Serial[" << _handle << "] " << portName << " start failed" << std::endl;
        }


        termios options{};

        tcgetattr(_handle, &options);
        std::cout<<options.c_oflag<<" "<<options.c_cflag << " "<<options.c_lflag<<std::endl;
        cfsetispeed(&options, baudrate);
        cfsetospeed(&options, baudrate);

        options.c_lflag  &= ~(ICANON | ECHO | ECHOE | ISIG);  /*Input*/
        options.c_oflag  &= ~OPOST;   /*Output*/

        switch (parity) {
// 无校验
            case 0:
                options.c_cflag &= ~PARENB;//PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag &= ~INPCK;//INPCK：使奇偶校验起作用
                break;
// 设置奇校验
            case 1:
                options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag |= PARODD;//PARODD：若设置则为奇校验,否则为偶校验
                options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
                options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
                break;
// 设置偶校验
            case 2:
                options.c_cflag |= PARENB;//PARENB：产生奇偶位，执行奇偶校验
                options.c_cflag &= ~PARODD;//PARODD：若设置则为奇校验,否则为偶校验
                options.c_cflag |= INPCK;//INPCK：使奇偶校验起作用
                options.c_cflag |= ISTRIP;//ISTRIP：若设置则有效输入数字被剥离7个字节，否则保留全部8位
                break;
            default:
                throw_with_trace(std::runtime_error, "unknown parity.");
        }
        switch (databit) {
            case 5:
                options.c_cflag &= ~CSIZE;//屏蔽其它标志位
                options.c_cflag |= CS5;
                break;
            case 6:
                options.c_cflag &= ~CSIZE;//屏蔽其它标志位
                options.c_cflag |= CS6;
                break;
            case 7:
                options.c_cflag &= ~CSIZE;//屏蔽其它标志位
                options.c_cflag |= CS7;
                break;
            case 8:
                options.c_cflag &= ~CSIZE;//屏蔽其它标志位
                options.c_cflag |= CS8;
                break;
            default:
                throw_with_trace(std::runtime_error, "unkown databit.");
        }

        switch (stopbit) {
            case 0:
                options.c_cflag &= ~CSTOPB;//CSTOPB：使用1位停止位
                break;
            case 2:
                options.c_cflag |= CSTOPB;//CSTOPB：使用2位停止位
                break;
            default:
                throw_with_trace(std::runtime_error, "unkown stopbit.");
        }
        std::cout<<(int)options.c_cc[VMIN]<<" "<<(int)options.c_cc[VTIME]<<std::endl;
        tcsetattr(_handle, TCSANOW, &options);

    }


    ~Serial() { close(_handle); }


    ssize_t Send(const unsigned char *data, int dataLenth) const {
        ssize_t ret = write(_handle, data, dataLenth);
        if (ret <= 0) {
            throw_with_trace(std::runtime_error, "can not send msg successfully");
        } else {
            LOG(INFO) << "sent " << ret << " bits" << std::endl;
        }
        return ret;
    }

    ssize_t Recv(uint8_t *data, int dataMaxLenth) const {
        return read(_handle, data, dataMaxLenth);
    }
};


