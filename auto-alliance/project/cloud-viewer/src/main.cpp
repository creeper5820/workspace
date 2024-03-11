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
    for (int x = 0; x < 50; x++)
        for (int y = 0; y < 50; y++)
            for (int z = 0; z < 50; z++) {
                box_src->push_back(PointType(
                    static_cast<float>(x) / 50,
                    static_cast<float>(y) / 50,
                    static_cast<float>(z) / 50));
            }

    auto rotation = Eigen::Affine3f { Eigen::AngleAxisf(M_PI_4, Eigen::Vector3f::UnitZ()) };
    auto translation = Eigen::Affine3f { Eigen::Translation3f { -0.5, -0.5, -0.5 } };
    // auto transform = (rotation * translation).matrix();
    auto transform = rotation * translation;
    pcl::transformPointCloud(*box_src, *box_src, transform);

    for (pcl::PointCloud<PointType>::iterator it = box_src->begin(); it < box_src->end(); it++) {
        if (1) {
            box_src->erase(it);
        }
    }

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