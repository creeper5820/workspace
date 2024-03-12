//
// Created by s on 23-7-29.
//

#include <tf/transform_datatypes.h>
#include "Navigator.h"

Navigator::Navigator(ros::NodeHandle &nh) :
        current_time(0), last_time(0),
        xPID(5, 0.1, 0, 1, -1),
        yPID(5, 0.1, 0, 1, -1),
        yawPID(1, 0.01, 0, 1, -1) {
    const std::string ns = "/nav/";
    nh.getParam(ns + "odomTopic", odomTopic);
    nh.getParam(ns + "velTopic", velTopic);
    nh.getParam(ns + "goalTopic", goalTopic);
    nh.getParam(ns + "init_x", init_x);
    nh.getParam(ns + "init_y", init_y);
    nh.getParam(ns + "static_map_url", static_map_url);
    static_map = cv::imread(static_map_url, 0);

    if (!static_map.empty()) {
        ROS_INFO("loaded static map with %dX%d", static_map.cols, static_map.rows);
    } else {
        ROS_INFO("load static map failed from %s", static_map_url.c_str());
    }

    sideSub = nh.subscribe<std_msgs::Bool>("side", 1, [this](auto &&PH1) {
        sideCallback(std::forward<decltype(PH1)>(PH1));
    });
    targetSub = nh.subscribe<control_msgs::Command>("cmd", 1, [this](auto &&PH1) {
        targetCallback(std::forward<decltype(PH1)>(PH1));
    });
    odomSub = nh.subscribe<nav_msgs::Odometry>(odomTopic, 1,
                                               [this](auto &&PH1) {
                                                   odomCallback(std::forward<decltype(PH1)>(PH1));
                                               });
    velPub = nh.advertise<geometry_msgs::Twist>(velTopic, 1);
    goalPub = nh.advertise<geometry_msgs::PoseStamped>(goalTopic, 1);
    staticMapPub = nh.advertise<nav_msgs::OccupancyGrid>("/static_map", 1);

    clearMapClient = nh.serviceClient<std_srvs::Empty>("/move_base/clear_costmaps");
    staticMapService = nh.advertiseService("/static_map", &Navigator::staticMapCallback, this);
    current_vel.setZero();
    current_pos.setZero();
    last_pos.setZero();
    last_vel.setZero();
    goal.header.frame_id = "field";
    goal.header.seq = 0;
    static_map_msg.header.frame_id = "field";
    static_map_msg.header.seq = 0;
    convertToOccupancyGrid(static_map, static_map_msg, 0.05);
    int side = 0;
    nh.getParam(ns + "preset_side", side);
    _side = static_cast<Side>(side);

    timer = nh.createTimer(ros::Rate(3), &Navigator::clearMapCallback, this);

}

void Navigator::loop() {
//    geometry_msgs::Twist cmd;
//    while (true) {
//        if (!stateQueue.empty()) {
//            auto &state = stateQueue.front();
//            cmd.angular.z = 0;
//            cmd.linear.x = xPID.calc(state.linear.x - current_pos(0));
//            cmd.linear.y = yPID.calc(state.linear.y - current_pos(1));
//            double ex = state.linear.x - current_pos(0);
//            double ey = state.linear.y - current_pos(1);
//            if ((ex * ex + ey * ey) < 0.005) {
//                stateQueue.pop();
//            }
//        }
//    }
}

void Navigator::odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg) {
    last_time = current_time;
    last_vel = current_vel;
    last_pos = current_pos;
    current_time = odomMsg->header.stamp.toSec();


    current_pos = Eigen::Vector2d(init_x + odomMsg->pose.pose.position.x,
                                  init_y + odomMsg->pose.pose.position.y);


    current_vel = Eigen::Vector2d(odomMsg->twist.twist.linear.x,
                                  odomMsg->twist.twist.linear.y);

    pose = odomMsg->pose.pose.orientation;
    pose.x = 0;
    pose.y = 0;
    normalize(pose);

}

void Navigator::targetCallback(const control_msgs::Command::ConstPtr &cmdMsg) {
    goal.header.seq++;
    goal.header.stamp = ros::Time::now();
//    ROS_INFO("%f, %f", cmdMsg->x, cmdMsg->y);
    if (last_cmd.x == cmdMsg->x && last_cmd.y == cmdMsg->y) {
        return;
    }
    ROS_INFO("target receive");

    switch (cmdMsg->key) {
        case 'A': {
            ROS_INFO("goal set");
            if (_side == BLUE) {
                goal.pose.position.x = 28 - cmdMsg->x;
                goal.pose.position.y = 15 - cmdMsg->y;
            } else if (_side == RED) {
                goal.pose.position.x = cmdMsg->x;
                goal.pose.position.y = cmdMsg->y;
            }

            goal.pose.orientation = pose;

            goalPub.publish(goal);
        }break;
        default:{

        }
    }
    last_cmd = *cmdMsg;
}

void Navigator::initField() {
    geometry_msgs::TransformStamped filed2init, body2real;

    filed2init = tf2::eigenToTransform(
            Eigen::Translation3d(init_x, init_y, 0.5) *
            Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX()) *
            Eigen::AngleAxisd(0, Eigen::Vector3d::UnitZ()));

    normalize(filed2init.transform.rotation);
    filed2init.header.frame_id = "field";
    filed2init.child_frame_id = "camera_init";
    ROS_INFO("init side %d", _side);
    staticBroadcaster.sendTransform(filed2init);

    body2real = tf2::eigenToTransform(
            Eigen::Translation3d(0, 0, 0) * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX())
    );
    normalize(body2real.transform.rotation);
    body2real.header.frame_id = "body";
    body2real.child_frame_id = "foot_print";
    staticBroadcaster.sendTransform(body2real);
}

void Navigator::sideCallback(const std_msgs::Bool::ConstPtr &sideMsg) {
    if (!_sideUpdated) {
        _sideUpdated = true;
        ROS_INFO("load side from stm32");
        _side = static_cast<Side>(sideMsg->data);
        ROS_INFO("side %d", _side);
    }

}

void Navigator::clearMapCallback(const ros::TimerEvent &e) {
    std_srvs::Empty empty{};
    clearMapClient.call(empty);
    static_map_msg.header.seq++;
    static_map_msg.header.stamp = ros::Time::now();
    staticMapPub.publish(static_map_msg);
}

bool Navigator::staticMapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res) {
    static_map_msg.header.seq++;
    static_map_msg.header.stamp = ros::Time::now();
    res.map = static_map_msg;
    ROS_INFO("map send %d", res.map.data.size());
    return true;
}

