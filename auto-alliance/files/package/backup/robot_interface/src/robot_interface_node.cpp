#include "RobotInterface.h"

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "serial");
    ros::NodeHandle nh;
    ros::Rate rate(100);
    RobotInterface dealer(nh);
    while(ros::ok()){
        ros::spinOnce();
        rate.sleep();
    }
}