//
// Created by soulde on 2023/6/24.
//
#include"env_sensing/MapGeneratorROS.h"


int main(int argc, char **argv) {
    ros::init(argc, argv, "env_sensing");

    ros::NodeHandle nh;
    MapGeneratorROS mapGeneratorRos(nh);

    ros::spin();

    return 0;
}
