#include <ros/ros.h>

#include "beginner_tutorials/add_server.h"

bool add(
    beginner_tutorials::add_server::Request& request,
    beginner_tutorials::add_server::Response& response)
{
    response.sum = request.first + request.second;

    ROS_INFO("request: first=%ld, second=%ld", (long int)request.first, (long int)request.second);
    ROS_INFO("response: sum= %ld", (long int)response.sum);

    return true;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "add_server");

    auto node = ros::NodeHandle();
    auto server = ros::ServiceServer(node.advertiseService("add", add));

    ROS_INFO("prepared");

    ros::spin();

    return 0;
}