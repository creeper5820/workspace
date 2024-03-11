#include <ros/ros.h>
#include <std_msgs/String.h>

void callback(const std_msgs::String::ConstPtr& msg)
{
    ROS_INFO("This is [%s]", msg->data.c_str());
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "listener");

    auto node = ros::NodeHandle();

    // auto callback = [](const std_msgs::String::ConstPtr& msg) -> void {};

    auto sub = ros::Subscriber(node.subscribe("chatter", 1000, callback));

    ros::spin();

    return 0;
}