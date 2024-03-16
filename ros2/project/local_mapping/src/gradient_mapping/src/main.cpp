#include <algorithm>
#include <functional>
#include <utility>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/detail/string__struct.hpp>
#include <std_msgs/msg/string.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"

#include "../include/gradient_mapping/transform.hpp"

using std::placeholders::_1;
using MessageType = livox_ros_driver2::msg::CustomMsg;

class GradientMapping : public rclcpp::Node {
public:
    GradientMapping()
        : Node("gradient_mapping")
        , transform_(Eigen::Affine3f { Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitZ()) })
    {
        subscription_ = this->create_subscription<MessageType>(
            "/livox/lidar", 10,
            std::bind(&GradientMapping::topic_callback, this, _1));
    }

private:
    rclcpp::Subscription<MessageType>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    Transform transform_;

    void topic_callback(const MessageType& msg)
    {
        transform_.update_cloud(msg);

        auto map = transform_.make_map();

        RCLCPP_INFO(this->get_logger(), "time_offset: '%zu', points: '%zu'", msg.timebase, transform_.size());
    }
};

int main(int argc, char* argv[])
{

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<GradientMapping>());
    rclcpp::shutdown();
    return 0;
}
