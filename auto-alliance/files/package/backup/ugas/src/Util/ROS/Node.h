#pragma once
/*
Creation Date: 2023/06/23
Latest Update: 2023/06/23
Developer(s): 22-Qzh
(C)Copyright: NJUST.Alliance - All rights reserved
Header Functions:
- 在一个独立线程中运行ROS2 Node
*/

#include "config.h"

#if ENABLE_ROS

#include <thread>
#include <ros/ros.h>

namespace ros_util {
    class Node{
    public:
        Node(ros::NodeHandle& nh)
        ~Node() override = default;

        tf2_ros::TransformBroadcaster tf_broadcaster_;

        rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr point_pub_;
        rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    };

    inline std::thread node_thread_;
    inline std::weak_ptr<Node> node_;

    inline void init(int argc, char* argv[]) {
        node_thread_ = std::thread([argc, argv](){
            rclcpp::init(argc, argv);
            auto node = std::make_shared<Node>();
            node_ = node;
            rclcpp::spin(node);
            rclcpp::shutdown();
        });
        node_thread_.detach();
    }
}

#else // ENABLE_ROS

#include <ros/ros.h>
#include <control_msgs/Command.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Bool.h>
#include "RobotInterface.h"

namespace ros_util {
    inline ros::Publisher goalPublisher, uwbPublisher, sidePublisher;
    inline ros::Subscriber twistSubscriber;
    inline control_msgs::Command command;
    inline geometry_msgs::PoseStamped goal, uwb;

    inline void twistCallback(const geometry_msgs::Twist::ConstPtr &msg) {

        moveControlFrame.x = static_cast<float>(msg->linear.x);
        moveControlFrame.y = -static_cast<float>(msg->linear.y);
        moveControlFrame.yaw = -static_cast<float>(msg->angular.z);
        isMoveControlFrameUpdated = true;
    }

    inline void init(int argc, char *argv[]) {
        ros::init(argc, argv, "ugas");
        ros::NodeHandle nh;
        goalPublisher = nh.advertise<control_msgs::Command>("/cmd", 1);
        uwbPublisher = nh.advertise<geometry_msgs::PoseStamped>("/UWB", 1);
        sidePublisher = nh.advertise<std_msgs::Bool>("/side", 1);


        twistSubscriber = nh.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, twistCallback);
        command.header.frame_id = "field";
        command.header.seq = 0;
        uwb.header = command.header;

        ros::Rate rate(20);
        while (ros::ok()) {
            ros::spinOnce();
            rate.sleep();
        }
        return;

    }
}

#endif // ENABLE_ROS
