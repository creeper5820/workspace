#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <eigen3/Eigen/Dense>

#include <boost/smart_ptr/shared_ptr.hpp>

#include <cmath>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

int main()
{
    using PointType = pcl::PointXYZ;

    auto box_src = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

    // point cloud
    for (int x = 0; x < 10; x++)
        for (int y = 0; y < 10; y++)
            for (int z = 0; z < 10; z++) {
                box_src->push_back(PointType(
                    static_cast<float>(x) / 10,
                    static_cast<float>(y) / 10,
                    static_cast<float>(z) / 10));
            }

    // rotation
    auto rotation = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
    double angle_x = M_PI / 4;
    rotation(1, 1) = std::cos(angle_x);
    rotation(1, 2) = -std::sin(angle_x);
    rotation(2, 1) = std::sin(angle_x);
    rotation(2, 1) = std::cos(angle_x);

    auto rotation2 = Eigen::AngleAxisf();
    // pcl::transformPointCloud(*box_src, *box_src, rotation);

    // translation
    auto cloud_center = Eigen::Vector4f();
    pcl::compute3DCentroid(*box_src, cloud_center);
    auto translation = Eigen::Matrix4f(Eigen::Matrix4f::Identity());
    auto transform = rotation * translation;
    translation(0, 3) = -cloud_center[0];
    translation(1, 3) = -cloud_center[1];
    translation(2, 3) = -cloud_center[2];
    pcl::transformPointCloud(*box_src, *box_src, transform);

    // plane
    auto coefficient = pcl::ModelCoefficients();
    coefficient.values.push_back(0);
    coefficient.values.push_back(0);
    coefficient.values.push_back(1);
    coefficient.values.push_back(0);

    auto view = boost::shared_ptr<pcl::visualization::PCLVisualizer>(new pcl::visualization::PCLVisualizer());
    view->setBackgroundColor(0, 0, 0);
    view->addPointCloud<PointType>(box_src, "box");
    view->addPlane(coefficient);
    view->addCoordinateSystem(0.5);

    view->spin();

    return 0;
}