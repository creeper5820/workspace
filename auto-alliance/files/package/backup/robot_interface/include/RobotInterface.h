//
// Created by soulde on 2023/3/4.
//

#ifndef SERIAL_SERIALFRAMEDEALER_H
#define SERIAL_SERIALFRAMEDEALER_H
#define float32_t float
#define float64_t double

#include <thread>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int8.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include <std_srvs/SetBool.h>
#include <Eigen/Eigen>
#include "control_msgs/Command.h"

#include "Serial.h"


class RobotInterface {

private:
    static constexpr unsigned char HEAD = 0xFE;

    enum class SendPackageID : unsigned char {
        GIMBAL = 0x01,
        MOVE = 0x02,
        GNSS = 0x03,
        HEARTBEAT = 0x41

    };

    enum class RecvPackageID : unsigned char {
        GIMBAL = 0x81,
        ODOM = 0x82,
        GOAL = 0x83,
        ENEMY = 0x84,
        UWB = 0x85,
        BUFF = 0xC1
    };
    enum Enemy {
        UNKNOWN = -1,
        BLUE = 0,
        RED = 1,

    };
#pragma pack(1)
    struct Header {
        const uint8_t head = HEAD;
        uint8_t id = 0x00;
        uint8_t crc8 = 0x00;
    };

    struct MoveControlFrame {
        Header header;
        float32_t x = 0.;
        float32_t y = 0.;
        float32_t yaw = 0.;
        uint8_t crc8 = 0x0;

        MoveControlFrame() {
            header.id = static_cast<unsigned char>(SendPackageID::MOVE);
        }

    } moveControlFrame;

    struct GimbalControlFrame {
        Header header;
        float32_t pitch = 0.;
        float32_t yaw = 0.;
        uint8_t fire = 0;
        uint8_t crc8 = 0x0;

        GimbalControlFrame() {
            header.id = static_cast<unsigned char>(SendPackageID::GIMBAL);
        }
    } gimbalControlFrame;

    struct HeartBeatFrame {
        Header header;

        HeartBeatFrame() {
            header.id = static_cast<unsigned char>(SendPackageID::HEARTBEAT);
        }
    } heartBeatFrame;

    struct GNSSFrame {
        Header header;
        float32_t lon;
        float32_t lat;
        float32_t alt;
        uint8_t crc8 = 0x0;

        GNSSFrame() {
            header.id = static_cast<unsigned char>(SendPackageID::GNSS);
        }
    } GNSSFrame;

    struct GimbalFeedbackFrame {
        Header header;
        float32_t pitch = 0.;
        float32_t yaw = 0.;
        float32_t fire = 0.;
        uint8_t crc8 = 0x0;
    } gimbalFeedbackFrame;

    struct OdomFeedbackFrame {
        Header header;
        float32_t x = 0.;
        float32_t vx = 0.;
        float32_t y = 0.;
        float32_t vy = 0.;
        float32_t yaw = 0.;
        float32_t wy = 0.;
        uint8_t crc8 = 0x0;
    } odomFeedbackFrame;

    struct GoalFeedbackFrame {
        Header header;
        float32_t x;
        float32_t y;
        uint8_t signal;
        uint8_t crc8 = 0x0;
    } goalFeedbackFrame;

    struct EnemyFeedbackFrame {
        Header header;
        uint8_t isRed;
        uint8_t crc8 = 0x0;
    } enemyFeedbackFrame;

    struct UWBFeedbackFrame {
        Header header;
        float32_t x;
        float32_t y;
        uint8_t crc8 = 0x0;
    } uwbFeedbackFrame;
#pragma pack()
    Serial *serial;
    std::string serialName;
    std::string subTopic, goalTopic, GNSSTopic = "fix";
    std::string fixFrame = "world", robotFrame = "base", gimbalFrame = "gimbal", odomFrame = "base_odom";
    bool useBaseOdomOnly = true;
    Eigen::Translation3d base2gimbalTrans;
    ros::NodeHandle nodeHandle;
    control_msgs::Command command;
    nav_msgs::Odometry odom;
    geometry_msgs::TransformStamped base2gimbal, world2base;
    geometry_msgs::PoseStamped goal, uwb;
    ros::Publisher goalPublisher, uwbPublisher, odomPublisher, sidePublisher;
    ros::Subscriber twistSubscriber, GNSSSubscriber;



    tf2_ros::TransformBroadcaster broadcaster;
    tf2_ros::StaticTransformBroadcaster staticBroadcaster;

    std::thread *recvThread;

    double init_x, init_y;
    void twistCallback(const geometry_msgs::Twist::ConstPtr &msg);

    void GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr &msg);

    [[noreturn]] void serialRecv();

public:
    RobotInterface() = delete;

    explicit RobotInterface(ros::NodeHandle &nh);

    ~RobotInterface() {
        delete serial;
        delete recvThread;
    }

    RobotInterface(RobotInterface &serialFrameDealer) = delete;

};


#endif //SERIAL_SERIALFRAMEDEALER_H
