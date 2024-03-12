//
// Created by soulde on 2023/3/4.
//

#include "RobotInterface.h"
#include "CRC/CRC.h"
#include "Serial.h"
#include <eigen_conversions/eigen_msg.h>

RobotInterface::RobotInterface(ros::NodeHandle &nh) : nodeHandle(nh) {
    nh.getParam("/robot_interface/serialName", serialName);

    nh.getParam("/robot_interface/subTopic", subTopic);
    nh.getParam("/robot_interface/goalTopic", goalTopic);
    nh.getParam("/robot_interface/GNSSTopic", GNSSTopic);

    nh.getParam("/robot_interface/fixFrame", fixFrame);
    nh.getParam("/robot_interface/robotFrame", robotFrame);
    nh.getParam("/robot_interface/gimbalFrame", gimbalFrame);

    nh.getParam("/robot_interface/base2gimbal_x", base2gimbalTrans.x());
    nh.getParam("/robot_interface/base2gimbal_y", base2gimbalTrans.y());
    nh.getParam("/robot_interface/base2gimbal_z", base2gimbalTrans.z());


    nh.getParam("/robot_interface/init_x", init_x);
    nh.getParam("/robot_interface/init_y", init_y);

    serial = new Serial(serialName.c_str());


    odom.header.seq = 0;
    odom.header.frame_id = fixFrame;
    odom.child_frame_id = odomFrame;

    command.header.frame_id = "field";
    command.header.seq = 0;

    base2gimbal.header.seq = 0;
    base2gimbal.header.frame_id = robotFrame;
    base2gimbal.child_frame_id = gimbalFrame;

    world2base.header.seq = 0;
    world2base.header.frame_id = fixFrame;
    world2base.child_frame_id = robotFrame;

    goal.header.seq = 0;
    goal.header.frame_id = fixFrame;

    uwb.header = goal.header;

//  4 publisher and 1 subscriber needed
    goalPublisher = nh.advertise<control_msgs::Command>(goalTopic, 1);
    uwbPublisher = nh.advertise<geometry_msgs::PoseStamped>("/UWB", 1);
    sidePublisher = nh.advertise<std_msgs::Bool>("/side", 1);


    twistSubscriber = nh.subscribe<geometry_msgs::Twist>(subTopic, 1,
                                                         [this](const geometry_msgs::Twist::ConstPtr &msg) {
                                                             this->twistCallback(msg);
                                                         });

// above


    recvThread = new std::thread(&RobotInterface::serialRecv, this);
    int timeout = 100;

}

void RobotInterface::twistCallback(const geometry_msgs::Twist::ConstPtr &msg) {
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&moveControlFrame.header), sizeof(Header));

    moveControlFrame.x = static_cast<float>(msg->linear.x);
    moveControlFrame.y = -static_cast<float>(msg->linear.y);
    moveControlFrame.yaw = -static_cast<float>(msg->angular.z);
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&moveControlFrame), sizeof(MoveControlFrame));

    ssize_t ret = serial->Send(reinterpret_cast<const unsigned char *>(&moveControlFrame), sizeof(MoveControlFrame));

    for (int i = 0; i < sizeof(MoveControlFrame); ++i) {
        std::cout << std::hex << " 0x" << static_cast<int>(*(reinterpret_cast<uint8_t *>(&moveControlFrame) + i));
    }
    std::cout << std::endl;
}

void RobotInterface::GNSSCallback(const sensor_msgs::NavSatFix::ConstPtr &msg) {
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&GNSSFrame.header), sizeof(Header));
    std::cout << "GNSS Data Send" << std::endl;
    GNSSFrame.lon = static_cast<float>(msg->longitude);
    GNSSFrame.lat = static_cast<float>(msg->latitude);
    GNSSFrame.alt = static_cast<float>(msg->altitude);
    Append_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&GNSSFrame), sizeof(GNSSFrame));

    ssize_t ret = serial->Send(reinterpret_cast<const unsigned char *>(&GNSSFrame), sizeof(GNSSFrame));

    for (int i = 0; i < sizeof(GNSSFrame); ++i) {
        std::cout << std::hex << " 0x" << static_cast<int>(*(reinterpret_cast<uint8_t *>(&GNSSFrame) + i));
    }
    std::cout << std::endl;
}

[[noreturn]] void RobotInterface::serialRecv() {
    Header header;
    while (true) {
        // wait until head equals HEAD
        while (true) {
            serial->Recv(reinterpret_cast<unsigned char *>(&header), 1);
            if (header.head == HEAD) {
                break;
            }
        }
        serial->Recv(reinterpret_cast<unsigned char *>(&header) + 1, sizeof(Header) - 1);
        if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&header), sizeof(Header))) {

            switch (static_cast<RecvPackageID>(header.id)) {
                case RecvPackageID::GIMBAL: {
                    memcpy(reinterpret_cast<unsigned char *>(&gimbalFeedbackFrame), &header, sizeof(Header));
                    serial->Recv(reinterpret_cast<unsigned char *>(&gimbalFeedbackFrame) + sizeof(Header),
                                 sizeof(GimbalFeedbackFrame) - sizeof(Header));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&gimbalFeedbackFrame),
                                              sizeof(GimbalFeedbackFrame))) {
                        base2gimbal = tf2::eigenToTransform(
                                base2gimbalTrans *
                                Eigen::AngleAxisd(gimbalFeedbackFrame.pitch / 180 * M_PI, Eigen::Vector3d::UnitY()) *
                                Eigen::AngleAxisd(gimbalFeedbackFrame.yaw / 180 * M_PI, Eigen::Vector3d::UnitZ())
                        );

                        base2gimbal.header.seq++;
                        base2gimbal.header.stamp = ros::Time::now();

//                        broadcaster.sendTransform(base2gimbal);
                    }
                }
                    break;
                case RecvPackageID::ODOM: {
                    memcpy(reinterpret_cast<unsigned char *>(&odomFeedbackFrame), &header, sizeof(Header));
                    static uint32_t count;
                    serial->Recv(reinterpret_cast<unsigned char *>(&odomFeedbackFrame) + sizeof(Header),
                                 sizeof(OdomFeedbackFrame) - sizeof(Header));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&odomFeedbackFrame),
                                              sizeof(OdomFeedbackFrame))) {
                        Eigen::Affine3d affinePos = Eigen::Translation3d(odomFeedbackFrame.x, odomFeedbackFrame.y, 0) *
                                                    Eigen::AngleAxisd(odomFeedbackFrame.yaw / 180 * M_PI,
                                                                      Eigen::Vector3d::UnitZ());
                        Eigen::Matrix<double, 6, 1> twist;
                        twist << odomFeedbackFrame.vx, odomFeedbackFrame.vy, 0, 0, 0, odomFeedbackFrame.wy;

                        tf::poseEigenToMsg(affinePos, odom.pose.pose);
                        tf::twistEigenToMsg(twist, odom.twist.twist);

                        odom.header.seq++;
                        odom.header.stamp = ros::Time::now();

                        odomPublisher.publish(odom);

                        world2base = tf2::eigenToTransform(affinePos);
                        world2base.header.seq++;
                        world2base.header.stamp = ros::Time::now();

//                        broadcaster.sendTransform(world2base);
                    }

                }
                    break;
                case RecvPackageID::GOAL: {

                    memcpy(reinterpret_cast<unsigned char *>(&goalFeedbackFrame), &header, sizeof(Header));
                    serial->Recv(reinterpret_cast<unsigned char *>(&goalFeedbackFrame) + sizeof(Header),
                                 sizeof(GoalFeedbackFrame) - sizeof(Header));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&goalFeedbackFrame),
                                              sizeof(GoalFeedbackFrame))) {
                        if (goalFeedbackFrame.x < 0.1) {
//                            ROS_INFO("get value too small");
                            break;
                        }
                        command.header.seq++;
                        command.key = goalFeedbackFrame.signal;
                        command.x = goalFeedbackFrame.x;
                        command.y = goalFeedbackFrame.y;

                        goalPublisher.publish(command);
                    }
                }
                    break;
                case RecvPackageID::ENEMY: {
                    memcpy(reinterpret_cast<unsigned char *>(&enemyFeedbackFrame), &header, sizeof(Header));
                    serial->Recv(reinterpret_cast<unsigned char *>(&enemyFeedbackFrame) + sizeof(Header),
                                 sizeof(enemyFeedbackFrame) - sizeof(Header));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&enemyFeedbackFrame),
                                              sizeof(enemyFeedbackFrame))) {
                        std_msgs::Bool sideMsg;
                        sideMsg.data = enemyFeedbackFrame.isRed;
                        sidePublisher.publish(sideMsg);
                        ROS_INFO("in enemy");
                    }

                }
                    break;
                case RecvPackageID::UWB: {
                    memcpy(reinterpret_cast<unsigned char *>(&uwbFeedbackFrame), &header, sizeof(Header));
                    serial->Recv(reinterpret_cast<unsigned char *>(&uwbFeedbackFrame) + sizeof(Header),
                                 sizeof(UWBFeedbackFrame) - sizeof(Header));
                    if (Verify_CRC8_Check_Sum(reinterpret_cast<unsigned char *>(&uwbFeedbackFrame),
                                              sizeof(UWBFeedbackFrame))) {
                        uwb.pose.position.x = uwbFeedbackFrame.x;
                        uwb.pose.position.y = uwbFeedbackFrame.y;
                        uwb.header.stamp = ros::Time::now();
                        uwb.header.seq++;

                        uwbPublisher.publish(uwb);
                    }
                }
                    break;
            }
        }
    }
}





