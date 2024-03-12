#pragma once
/*
Creation Date: 2023/04/21
Latest Update: 2023/08/02
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
与步兵的串口通讯
*/

#include <chrono>
#include <memory>
#include <thread>
#include <atomic>
#include <mutex>

#include "Util/Serial/CRC.h"
#include "Util/Serial/SerialUtil.h"
#include "Util/Debug/Log.h"
#include "Util/Parameter/Parameters.h"
#include "Util/ROS/Node.h"
#include "Util/ROS/RobotInterface.h"

class CBoardInfantry {
public:

#pragma pack(push, 1)
    struct DataSend {
        float yaw, pitch;
        uint16_t rect_x, rect_y;
        uint8_t color;
    };
    struct DataReceive {
        uint8_t self_color;              // 自身队伍颜色：1-红，2-蓝
        uint8_t preset_bullet_speed;     // 预设弹速，单位：m/s
        float bullet_speed;              // 实时弹速，单位：m/s
        uint8_t auto_scope_enabled;      // 操作手是否开启自瞄
    };
#pragma pack(pop)

    explicit CBoardInfantry(const char *portName) :
            serial_(portName, 115200, serial::Timeout::simpleTimeout(100000000)),
            recvFlag(true) {
        recv = std::make_unique<std::thread>(&CBoardInfantry::Receive, this);

    }

    ~CBoardInfantry() {
        recvFlag = false;
        recv->join();
    };


    // 向除哨兵外的地面兵种发送云台瞄准数据
    // 没有目标时调用
    void Send() {
        CRC::DjiCRC8Calculator::Append(gimbalControlFrame.header);
        gimbalControlFrame.yaw = 0.0f;
        gimbalControlFrame.pitch = 0.0f;
        gimbalControlFrame.fire = false;
        CRC::DjiCRC8Calculator::Append(gimbalControlFrame);
        serial_.write(reinterpret_cast<uint8_t *>(&gimbalControlFrame), sizeof(gimbalControlFrame));
        if (isMoveControlFrameUpdated) {
            std::cout<<"in"<<std::endl;
            isMoveControlFrameUpdated = false;
            CRC::DjiCRC8Calculator::Append(moveControlFrame.header);
            CRC::DjiCRC8Calculator::Append(moveControlFrame);
            serial_.write(reinterpret_cast<uint8_t *>(&moveControlFrame), sizeof(moveControlFrame));
        }
    }

    /*! 向除哨兵外的地面兵种发送云台瞄准数据
    * yaw pitch: 击中目标所需云台移动差值，单位使用弧度制，方向遵循右手定则
    */
    void Send(double yaw, double pitch) {
        CRC::DjiCRC8Calculator::Append(gimbalControlFrame.header);
        gimbalControlFrame.yaw = static_cast<float>(-yaw * 180.0 / parameters::Pi);
        gimbalControlFrame.pitch = static_cast<float>(-pitch * 180.0 / parameters::Pi);
        gimbalControlFrame.fire = true;
//        std::cout << gimbalControlFrame.yaw << ' ' << gimbalControlFrame.pitch << '\n';
        CRC::DjiCRC8Calculator::Append(gimbalControlFrame);
        serial_.write(reinterpret_cast<uint8_t *>(&gimbalControlFrame), sizeof(gimbalControlFrame));
        if (isMoveControlFrameUpdated) {
            isMoveControlFrameUpdated = false;
            CRC::DjiCRC8Calculator::Append(moveControlFrame.header);
            CRC::DjiCRC8Calculator::Append(moveControlFrame);
            serial_.write(reinterpret_cast<uint8_t *>(&moveControlFrame), sizeof(moveControlFrame));
        }
    }

//    /*! 向无人机发送云台瞄准数据
//    * \param yaw pitch 单位使用弧度制，方向遵循右手定则
//    */
//    void SendUAV(double yaw, double pitch) {
//        _sender.Data.yaw = -yaw;
//        _sender.Data.pitch = -pitch;
//        _sender.Send();
//    }


    [[noreturn]] void Receive() {
        while (recvFlag) {

            // wait until head equals HEAD
            while (true) {
                serial_.read(reinterpret_cast<unsigned char *>(&header), 1);
                if (header.head == HEAD) {
                    break;
                }
            }
            serial_.read(reinterpret_cast<unsigned char *>(&header) + 1, sizeof(Header) - 1);

            if (CRC::DjiCRC8Calculator::Verify(header)) {
                switch (static_cast<RecvPackageID>(header.id)) {
                    case RecvPackageID::GOAL: {

                        memcpy(reinterpret_cast<unsigned char *>(&goalFeedbackFrame), &header, sizeof(Header));
                        serial_.read(reinterpret_cast<unsigned char *>(&goalFeedbackFrame) + sizeof(Header),
                                     sizeof(GoalFeedbackFrame) - sizeof(Header));

                        if (CRC::DjiCRC8Calculator::Verify(goalFeedbackFrame)) {
//                            std::cout<<"in"<<std::endl;
                            if (goalFeedbackFrame.x < 0.1) {
//                            ROS_INFO("get value too small");
                                break;
                            }
//                            std::cout << "receive goal " << goalFeedbackFrame.x << goalFeedbackFrame.y
//                                      << (int)goalFeedbackFrame.signal << std::endl;
                            ros_util::command.header.seq++;
                            ros_util::command.key = goalFeedbackFrame.signal;
                            ros_util::command.x = goalFeedbackFrame.x;
                            ros_util::command.y = goalFeedbackFrame.y;

                            ros_util::goalPublisher.publish(ros_util::command);
                        }
                    }
                        break;
                    case RecvPackageID::ENEMY: {
                        memcpy(reinterpret_cast<unsigned char *>(&enemyFeedbackFrame), &header, sizeof(Header));
                        serial_.read(reinterpret_cast<unsigned char *>(&enemyFeedbackFrame) + sizeof(Header),
                                     sizeof(enemyFeedbackFrame) - sizeof(Header));

                        if (CRC::DjiCRC8Calculator::Verify(enemyFeedbackFrame)) {
                            std_msgs::Bool sideMsg;
                            sideMsg.data = enemyFeedbackFrame.isRed;
                            if (enemyFeedbackFrame.isRed)
                                enemy_color_ = ArmorColor::Red;
                            else
                                enemy_color_ = ArmorColor::Blue;
                            shooter_index_ = enemyFeedbackFrame.barrel;
                            ros_util::sidePublisher.publish(sideMsg);
                        }
                    }
                        break;
                    case RecvPackageID::UWB: {
                        memcpy(reinterpret_cast<unsigned char *>(&uwbFeedbackFrame), &header, sizeof(Header));
                        serial_.read(reinterpret_cast<unsigned char *>(&uwbFeedbackFrame) + sizeof(Header),
                                     sizeof(UWBFeedbackFrame) - sizeof(Header));
                        if (CRC::DjiCRC8Calculator::Verify(uwbFeedbackFrame)) {
                            ros_util::uwb.pose.position.x = uwbFeedbackFrame.x;
                            ros_util::uwb.pose.position.y = uwbFeedbackFrame.y;
                            ros_util::uwb.header.stamp = ros::Time::now();
                            ros_util::uwb.header.seq++;

                            ros_util::uwbPublisher.publish(ros_util::uwb);
                        }
                    }
                        break;
                }
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(1));

        }

    }

    [[nodiscard]] ArmorColor get_enemy_color() const {
        return enemy_color_;
    }

//
    [[nodiscard]] double get_bullet_speed() const {
        return parameters::AverageBulletSpeed30;
    }

    [[nodiscard]] uint8_t get_shooter_index() const {
        return shooter_index_;
    }
//
//    [[nodiscard]] bool get_auto_scope_enabled() const {
//        return auto_scope_enabled_;
//    }

private:
    serial::Serial serial_;
    std::unique_ptr<std::thread> recv;
    std::atomic<bool> recvFlag = true;
//    bool receive_succeed_ = false;
//
//    SerialUtil::SerialSender<DataSend, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> sender_;
//    SerialUtil::SerialReceiver<DataReceive, SerialUtil::Head<uint8_t, 0xff>, CRC::DjiCRC8Calculator> receiver_;
    std::atomic<ArmorColor> enemy_color_ = parameters::DefaultEnemyColor;
    std::atomic<std::uint8_t> shooter_index_ = 0;
//    double bullet_speed_ = parameters::DefaultBulletSpeed;
//    bool auto_scope_enabled_ = false;


};
