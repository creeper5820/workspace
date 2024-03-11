#include "beginner_tutorials/add_server.h"

#include <ros/ros.h>

#include <cstdlib>

int main(int c, char** v)
{
    ros::init(c, v, "add_client");

    if (3 != c) {
        ROS_INFO("usage: add_client first second");
        return 1;
    }

    auto node = ros::NodeHandle();
    auto client = ros::ServiceClient(node.serviceClient<beginner_tutorials::add_server>("add"));

    auto server = beginner_tutorials::add_server();
    server.request.first = atoll(v[1]);
    server.request.second = atoll(v[2]);

    if (client.call(server)) {
        ROS_INFO("sum: %ld", (long int)server.response.sum);
    } else {
        ROS_ERROR("failed");
        return 1;
    }

    return 0;
}