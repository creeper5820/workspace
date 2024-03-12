//
// Created by s on 23-7-29.
//
#include "Navigator.h"

int main(int argc, char **argv) {
    ros::init(argc, argv, "nav");
    ros::NodeHandle nh;
    Navigator navigator(nh);
    ros::Rate rate(100);

    while (ros::ok()) {
        ros::spinOnce();
        if (navigator.sideUpdated()) {
            break;
        }
        rate.sleep();
    }
    ROS_INFO("Field inited");
    navigator.initField();
    ros::spin();

    return 0;
}