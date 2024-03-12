//
// Created by s on 23-7-29.
//

#ifndef NAV_NAVIGATOR_H
#define NAV_NAVIGATOR_H

#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "nav_msgs/Odometry.h"
#include "nav_msgs/GetMap.h"
#include "std_msgs/Bool.h"
#include "std_srvs/Empty.h"
#include "Eigen/Eigen"
#include "queue"
#include "utility"
#include "tf2_ros/static_transform_broadcaster.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_eigen/tf2_eigen.h>
#include "thread"
#include "mutex"
#include "condition_variable"
#include "opencv2/opencv.hpp"
#include "PID.h"
#include "control_msgs/Command.h"

class Navigator {

public:
    explicit Navigator(ros::NodeHandle &nh);

    bool sideUpdated() const {
        return _sideUpdated;
    }

    void initField();

private:
    std::string targetTopic, odomTopic, velTopic, goalTopic, static_map_url;
    ros::Subscriber targetSub, odomSub, sideSub;
    cv::Mat static_map;
    ros::Publisher goalPub, velPub, staticMapPub;
    ros::ServiceClient clearMapClient;
    ros::ServiceServer staticMapService;
    tf2_ros::StaticTransformBroadcaster staticBroadcaster;

    ros::Timer timer;
    double init_x, init_y;
    bool _sideUpdated = false;
    control_msgs::Command last_cmd;
    geometry_msgs::PoseStamped goal;
    nav_msgs::OccupancyGrid static_map_msg;
    double current_time, last_time;
    Eigen::Vector2d current_pos, current_vel, last_pos, last_vel;

    geometry_msgs::Quaternion pose;

    PID xPID, yPID, yawPID;
//    enum class State {
//        WAIT,
//        MOVE,
//    } _state;
    enum Side {
        UNKNOWN = -1,
        RED = 0,
        BLUE = 1,
    } _side;
    //std::thread thread;
    std::mutex mutex;


    std::queue<geometry_msgs::Twist> stateQueue;

    void loop();

    void normalize(geometry_msgs::Quaternion &q) {
        double norm = sqrt(q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z);
        q.w /= norm;
        q.x /= norm;
        q.y /= norm;
        q.z /= norm;
    }

    static void convertToOccupancyGrid(cv::Mat &src, nav_msgs::OccupancyGrid &msg, float resolution) {
        std::vector<signed char> vec;
        //将图像文件转换为vector，并将数据范围缩放至0-100
        for (int i = src.rows - 1; i >= 0; i--) {
            uint8_t *p = src.ptr(i);
            for (int j = 0; j < src.cols; j++)
                msg.data.push_back((255 - p[j]) * 100 / 255);
        }

        msg.info.resolution = resolution;//地图分辨率

        msg.info.width = src.cols;
        msg.info.height = src.rows;

        msg.info.origin.position.x = 0;
        msg.info.origin.position.y = 0;
        msg.info.origin.position.z = 0;

    }

    void sideCallback(const std_msgs::Bool::ConstPtr &sideMsg);

    void odomCallback(const nav_msgs::Odometry::ConstPtr &odomMsg);

    void targetCallback(const control_msgs::Command::ConstPtr &cmdMsg);

    void clearMapCallback(const ros::TimerEvent &e);

    bool staticMapCallback(nav_msgs::GetMap::Request &req, nav_msgs::GetMap::Response &res);
};


#endif //NAV_NAVIGATOR_H
