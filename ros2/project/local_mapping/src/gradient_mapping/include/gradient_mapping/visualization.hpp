#pragma once

#include <memory>
#include <thread>

#include <eigen3/Eigen/Dense>

#include <pcl/features/normal_3d.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include "./transform.hpp"

namespace local {
class Visualization {
public:
    Visualization(std::string&& name)
        : view_(new pcl::visualization::PCLVisualizer(name))
    {
    }

    pcl::visualization::PCLVisualizer::Ptr operator->()
    {
        return view_;
    }

    void set_background_color(const double& r, const double& g, const double& b) const
    {
        view_->setBackgroundColor(r, g, b);
    }

    void spin_once(const std::shared_ptr<local::Transform::MapType>& map)
    {
        using namespace std::chrono_literals;

        auto center = Eigen::Vector3f(0, 0, 0);
        auto rotation = Eigen::Quaternionf(1, 0, 0, 0);

        for (int row = 0; row < map->rows(); row++)
            for (int col = 0; col < map->cols(); col++) {

                auto depth = (*map)(row, col);

                center = Eigen::Vector3f(row, col, depth / 2);

                view_->addCube(center, rotation,
                    1, 1, depth,
                    std::string("cube_") + std::to_string(row) + "_" + std::to_string(col));
            }

        while (!view_->wasStopped()) {
            view_->spinOnce(100);
            std::this_thread::sleep_for(100ms);
        }
    }

private:
    pcl::visualization::PCLVisualizer::Ptr view_;

}; // Visualization
} // local