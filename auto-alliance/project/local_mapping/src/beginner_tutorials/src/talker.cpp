#include <ros/ros.h>
#include <std_msgs/String.h>

#include <cstdint>
#include <sstream>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "talker");

    auto node_handle = ros::NodeHandle();
    auto chatter_publish = ros::Publisher(node_handle.advertise<std_msgs::String>("chatter", 1000));
    auto rate_loop = ros::Rate(10);

    uint32_t count = 0;

    while (ros::ok()) {

        // generate messages and info them
        auto msg = std_msgs::String();
        auto string_stream = std::stringstream();

        string_stream << "hello world" << count;
        msg.data = string_stream.str();

        ROS_INFO("%s", msg.data.c_str());

        // publish messages
        chatter_publish.publish(msg);

        ros::spinOnce();

        rate_loop.sleep();
        count++;
    }

    return 0;
}