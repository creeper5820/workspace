//
// Created by soulde on 2023/3/7.
//

#include "PathFollower.h"

PathFollower::PathFollower(ros::NodeHandle &handle) : pubThread(&PathFollower::loop, this), current_yaw(0),
                                                      current_time(0), last_time(0),
                                                      xPID(5, 0.1, 0, 0.5, -0.5),
                                                      yPID(5, 0.1, 0, 0.5, -0.5),
                                                      yawPID(1, 0.01, 0, 1, -1) {

    const std::string ns = "/path_follower/";
    handle.getParam(ns + "targetMode", targetMode);
    if (targetMode == DYNAMIC) {
        handle.getParam(ns + "targetTopic", targetTopic);
        targetSub = handle.subscribe<control_msgs::PositionCommand>(targetTopic, 1,
                                                                      [this](auto &&PH1) {
                                                                          targetCallback(
                                                                                  std::forward<decltype(PH1)>(PH1));
                                                                      });
    } else {
        handle.getParam(ns + "targetListUrl", targetsListUrl);
        ROS_INFO("url %s", targetsListUrl.c_str());
        cv::FileStorage fs(targetsListUrl, cv::FileStorage::READ);
        fs["target"] >> targetList;
        ROS_INFO("static target mode, %zu targets total", targetList.size());
    }
    handle.getParam(ns + "odomHasVel", odomHasVel);
    handle.getParam(ns + "odomTopic", odomTopic);
    handle.getParam(ns + "velTopic", velTopic);

    odomSub = handle.subscribe<nav_msgs::Odometry>(odomTopic, 1,
                                                   [this](auto &&PH1) {
                                                       odomCallback(std::forward<decltype(PH1)>(PH1));
                                                   });
    velPub = handle.advertise<geometry_msgs::Twist>(velTopic, 1);
    current_vel.setZero();
    current_pos.setZero();
    last_pos.setZero();
    last_vel.setZero();
    mainLoopCV.notify_all();
}

void PathFollower::publishCmd() {
    velPub.publish(twist);
}

void PathFollower::odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg) {
    last_time = current_time;
    last_vel = current_vel;
    last_pos = current_pos;
    current_time = odomMsg->header.stamp.toSec();
    current_pos = Eigen::Vector2d(odomMsg->pose.pose.position.x,
                                  odomMsg->pose.pose.position.y);
    if (odomHasVel) {
        current_vel = Eigen::Vector2d(odomMsg->twist.twist.linear.x,
                                      odomMsg->twist.twist.linear.y);
    } else {
        current_vel = (current_pos - last_pos) / (current_time - last_time);
    }
    pitch = odomMsg->twist.twist.angular.y;
    current_yaw = tf::getYaw(odomMsg->pose.pose.orientation);

}

void PathFollower::targetCallback(const control_msgs::PositionCommand::ConstPtr &cmdMsg) {
    //use k if necessary
//    kx_ = Eigen::Vector3d(cmdMsg->kx[0], cmdMsg->kx[1], cmdMsg->kx[2]);
//    kv_ = Eigen::Vector3d(cmdMsg->kv[0], cmdMsg->kv[1], cmdMsg->kv[2]);

    stateQueue.emplace(Eigen::Vector2d(cmdMsg->position.x, cmdMsg->position.y),
                       Eigen::Vector2d(cmdMsg->velocity.x, cmdMsg->velocity.y),
                       cmdMsg->yaw);
}

[[noreturn]] void PathFollower::loop() {
    std::unique_lock<std::mutex> lck(mtx);
    mainLoopCV.wait(lck);
    while (true) {
        if (targetMode == STATIC && stateQueue.empty()) {
            for (const auto &item: targetList) {
                stateQueue.emplace(Eigen::Vector2d(item.x, item.y), Eigen::Vector2d(0, 0), 0);
            }
        }
        calculateVel();
        publishCmd();
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
    }
}

void PathFollower::calculateVel() {
    auto state = stateQueue.front();

    auto errPos = state.pose - current_pos;
    auto errVel = state.vel - current_vel;
    twist.angular.z = 0;
    twist.linear.x = xPID.calc(errPos.x());
    twist.linear.y = yPID.calc(errPos.y());

//    if (abs(twist.angular.z) > 0.00) {
//        twist.linear.x = 0.00;
//    }

    ROS_INFO("current pos (%f, %f, %f)", current_pos.x(), current_pos.y(), current_yaw / 3.14 * 180);
    ROS_INFO("move to pos (%f, %f, %f), control value %f, %f", state.pose.x(), state.pose.y(),
             atan(errPos.y() / errPos.x()) / 3.14 * 180, twist.linear.x,
             twist.angular.z);
    if (errPos.norm() < 0.05) {
        stateQueue.pop();
    }
}


