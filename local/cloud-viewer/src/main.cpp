#include <pcl/common/centroid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <eigen3/Eigen/Dense>
#include <chrono>

#include <boost/smart_ptr/shared_ptr.hpp>

#include <cmath>
#include <iostream>
#include <memory>
#include <numbers>
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

    Eigen::Affine3f rotation = Eigen::Affine3f { Eigen::AngleAxisf(M_PI_4f, Eigen::Vector3f::UnitZ()) };
    Eigen::Affine3f translation = Eigen::Affine3f { Eigen::Translation3f { 0, 0, 1 } };
    Eigen::Matrix4f transform = (rotation * translation).matrix();

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
    view->addPlane({ coefficient });
    view->addCoordinateSystem(0.5);

    return 0;
}