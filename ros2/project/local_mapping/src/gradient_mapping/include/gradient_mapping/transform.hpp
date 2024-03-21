#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <vector>

#include <pcl/common/transforms.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <eigen3/Eigen/Dense>

#include "livox_ros_driver2/msg/custom_msg.hpp"
#include "livox_ros_driver2/msg/custom_point.hpp"

// auto rotation = Eigen::Affine3f { Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitZ()) };

// uint32 offset_time      # offset time relative to the base time
// float32 x               # X axis, unit:m
// float32 y               # Y axis, unit:m
// float32 z               # Z axis, unit:m
// uint8 reflectivity      # reflectivity, 0~255
// uint8 tag               # livox tag
// uint8 line              # laser number in lidar

// std_msgs/Header header    # ROS standard message header
// uint64 timebase           # The time of first point
// uint32 point_num          # Total number of pointclouds
// uint8  lidar_id           # Lidar device id number
// uint8[3]  rsvd            # Reserved use
// CustomPoint[] points      # Pointcloud data

namespace local {

class Transform {
public:
    using PointType = pcl::PointXYZ;
    using MapType = Eigen::Matrix<float, 30, 30>;

    Transform(const Eigen::Affine3f& transform)
        : transform_(transform)
        , cloud_(new pcl::PointCloud<PointType>())
    {
    }

    void update_cloud(const livox_ros_driver2::msg::CustomMsg& msg)
    {
        cloud_->clear();

        auto point = PointType();

        for (auto i : msg.points) {
            point.x = i.x;
            point.y = i.y;
            point.z = i.z;
            cloud_->push_back(point);
        }
    }

    std::shared_ptr<MapType> make_map()
    {
        auto map = std::make_shared<MapType>();
        pcl::transformPointCloud(*cloud_, *cloud_, transform_);

        for (auto i : cloud_->points) {
            if (1
                && is_in(i.x, -1.5, 1.5)
                && is_in(i.y, -1.5, 1.5)
                && is_in(i.z, -1.0, 3.0)) {
                (*map)(static_cast<int>((i.x + 1.5) * 10), static_cast<int>((i.y + 1.5) * 10)) += i.z;
            }
        }
        return map;
    }

    size_t size()
    {
        return cloud_->size();
    }

private:
    Eigen::Affine3f transform_;
    pcl::PointCloud<PointType>::Ptr cloud_;

    template <typename T1, typename T2>
    bool is_in(const T1& value, const T2&& left, const T2&& right)
    {
        return value > left && value < right;
    }

}; // Transform
} // local