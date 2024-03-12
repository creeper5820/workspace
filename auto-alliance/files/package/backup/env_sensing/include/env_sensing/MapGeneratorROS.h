//
// Created by soulde on 2023/6/24.
//

#ifndef ENV_SENSING_MAPGENERATORROS_H
#define ENV_SENSING_MAPGENERATORROS_H

#include "MapGenerator.h"
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <tf2_eigen/tf2_eigen.h>
#include <grid_map_ros/grid_map_ros.hpp>

#include <grid_map_msgs/GridMap.h>

class MapGeneratorROS {
public:
    explicit MapGeneratorROS(ros::NodeHandle& nh);


private:
    std::string pcTopic, imuTopic, mapTopic, filterChainParametersName;
    MapGenerator generator;

    ros::Subscriber pcSubscriber;
    ros::Publisher mapPublisher, pcPublisher, pcDebugPublisher;

    void pcCallback(sensor_msgs::PointCloud2::ConstPtr &pc2);
};


#endif //ENV_SENSING_MAPGENERATORROS_H
