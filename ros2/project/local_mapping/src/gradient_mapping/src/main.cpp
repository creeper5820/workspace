#include <algorithm>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <utility>

#include <rclcpp/publisher.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/float32_multi_array.hpp>
#include <std_msgs/msg/string.hpp>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"
#include "messages/msg/custom_msg.hpp"

#include "../include/gradient_mapping/transform.hpp"
// #include "../include/gradient_mapping/visualization.hpp"

using std::placeholders::_1;
using namespace std::chrono_literals;

class GradientMapping : public rclcpp::Node {
public:
    using Status = struct {
        uint32_t time_base;
        uint32_t livox_count;
    };

    GradientMapping()
        : Node("gradient_mapping")
        , transform_(Eigen::Affine3f { Eigen::AngleAxisf(M_PI, Eigen::Vector3f::UnitX()) })
    // , view_("view")
    {
        // subscribe
        subscription_
            = this->create_subscription<livox_ros_driver2::msg::CustomMsg>(
                "/livox/lidar", 10, std::bind(&GradientMapping::livox_callback, this, _1));

        // publish
        custom_
            = this->create_publisher<messages::msg::CustomMsg>(
                "/map/two", 10);

        // time callback
        time_custom_
            = this->create_wall_timer(
                100ms, std::bind(&GradientMapping::custom_allback, this));
        time_log_
            = this->create_wall_timer(
                1000ms, std::bind(&GradientMapping::log_callback, this));
        // time_visualization_
        //     = this->create_wall_timer(
        //         1ms, std::bind(&GradientMapping::visualization_callback, this));
    }

private:
    rclcpp::Subscription<livox_ros_driver2::msg::CustomMsg>::SharedPtr subscription_;
    rclcpp::Publisher<messages::msg::CustomMsg>::SharedPtr custom_;

    rclcpp::TimerBase::SharedPtr time_custom_;
    rclcpp::TimerBase::SharedPtr time_log_;
    // rclcpp::TimerBase::SharedPtr time_visualization_;

    local::Transform transform_;
    // local::Visualization view_;

    Status status_ = { 0, 0 };

    void livox_callback(const livox_ros_driver2::msg::CustomMsg& msg)
    {
        transform_.update_cloud(msg);

        status_.time_base = msg.timebase;
        status_.livox_count++;
    }

    void custom_allback()
    {
        auto map = transform_.make_map();
    }

    // void visualization_callback()
    // {
    //     auto map = std::make_shared<local::Transform::MapType>();

    //     (*map)(2, 4) = 1;
    //     (*map)(2, 8) = 2;
    //     (*map)(8, 4) = 5.3;
    //     (*map)(1, 0) = 1;

    //     view_.spin_once(map);
    // }

    void log_callback()
    {
        RCLCPP_INFO(this->get_logger(), "time: '%u', receive: '%u''", status_.time_base, status_.livox_count);
    }
};

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);

    rclcpp::spin(std::make_shared<GradientMapping>());

    rclcpp::shutdown();

    return 0;
}
