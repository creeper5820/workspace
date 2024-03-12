//
// Created by soulde on 2023/3/7.
//

#ifndef RM_WS_PATHFOLLOWER_H
#define RM_WS_PATHFOLLOWER_H

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <control_msgs/PositionCommand.h>
#include <Eigen/Eigen>
#include <queue>
#include <utility>
#include <tf/transform_datatypes.h>
#include <thread>
#include <mutex>
#include <condition_variable>
#include <opencv2/opencv.hpp>
#include "PID.h"
class PathFollower {
public:
    PathFollower() = delete;

    explicit PathFollower(ros::NodeHandle &handle);

    void publishCmd();

private:
    enum TARGET_MODE{
        DYNAMIC = 0,
        STATIC = 1
    };
    std::string targetTopic, odomTopic, velTopic;
    std::string targetsListUrl;
    std::vector<cv::Point2d> targetList;
    bool odomHasVel = false;
    int targetMode = STATIC;
    ros::Subscriber targetSub, odomSub;
    ros::Publisher velPub;

    PID xPID, yPID, yawPID;

    struct State {
        Eigen::Vector2d pose, vel;
        double yaw;

        State(Eigen::Vector2d pose, Eigen::Vector2d vel, double yaw) : pose(std::move(pose)), vel(std::move(vel)),
                                                                       yaw(yaw) {};
    };

    std::mutex mtx;
    std::condition_variable mainLoopCV;

    std::queue<State> stateQueue;
    double current_time, last_time;
    Eigen::Vector2d current_pos, current_vel,last_pos, last_vel;
    double current_yaw{};
    double pitch;

    geometry_msgs::Twist twist;

    std::thread pubThread;

    [[noreturn]] void loop();

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);

    void targetCallback(const control_msgs::PositionCommand::ConstPtr &cmdMsg);

    void calculateVel();


};


#endif //RM_WS_PATHFOLLOWER_H
