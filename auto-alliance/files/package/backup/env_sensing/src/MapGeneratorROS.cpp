//
// Created by soulde on 2023/6/24.
//

#include "env_sensing/MapGeneratorROS.h"

MapGeneratorROS::MapGeneratorROS(ros::NodeHandle &nh) : generator(3) {
    pcTopic = nh.param<std::string>("/env_sensing/pcTopic", "/livox/lidar");
    mapTopic = nh.param<std::string>("/env_sensing/mapTopic", "/grid_map");

    nh.getParam("/env_sensing/pcTopic", pcTopic);
    nh.getParam("/env_sensing/mapTopic", mapTopic);

    pcSubscriber = nh.subscribe<sensor_msgs::PointCloud2>(pcTopic, 1, [this](sensor_msgs::PointCloud2::ConstPtr msg) {
        MapGeneratorROS::pcCallback(msg);
    });
    ROS_INFO("suscribe to %s", pcTopic.c_str());
    mapPublisher = nh.advertise<grid_map_msgs::GridMap>(mapTopic, 1);

    pcPublisher = nh.advertise<sensor_msgs::PointCloud2>("/local_map", 1);

    pcDebugPublisher = nh.advertise<sensor_msgs::PointCloud2>("/debug_cloud", 1);

}


void MapGeneratorROS::pcCallback(sensor_msgs::PointCloud2::ConstPtr &pc2) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pc(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc2, *pc);
    generator.inputPointCloud(pc);
    int ret = generator.genMap();

    if (ret < 0) {
        return;
    }
    grid_map::GridMap outMap;
    grid_map_msgs::GridMap map_msg;
    grid_map::GridMapRosConverter::toMessage(generator.getMap(), map_msg);
    map_msg.info.header.frame_id = "body";
    map_msg.info.header.stamp = pc2->header.stamp;
    mapPublisher.publish(map_msg);


    sensor_msgs::PointCloud2 pc2_debug_msg;
//    pcl::toROSMsg(*generator.getDebugCloud(), pc2_debug_msg);
//    std::cout<<pc2_debug_msg.data.size()<<std::endl;
//    pc2_debug_msg.header.frame_id = "body";
//    pc2_debug_msg.header.stamp = pc2->header.stamp;
//    pcDebugPublisher.publish(pc2_debug_msg);

    sensor_msgs::PointCloud2 pc2_msg;
    pcl::toROSMsg(*generator.getCloud(), pc2_msg);
    pc2_msg.header.frame_id = "body";
    pc2_msg.header.stamp = pc2->header.stamp;
    pcPublisher.publish(pc2_msg);
}
