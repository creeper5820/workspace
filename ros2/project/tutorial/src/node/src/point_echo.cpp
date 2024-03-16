#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

#include <functional>
#include <memory>

using std::placeholders::_1;

class LocalMapping : public rclcpp::Node {
public:
    LocalMapping()
        : Node("minimal_subscriber")
    {
        subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/livox/lidar", 10,
            std::bind(&LocalMapping::callback, this, _1));
    }

private:
    void callback(const std_msgs::msg::String& msg) const
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }

    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<LocalMapping>());
    rclcpp::shutdown();

    return 0;
}