#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

#include <boost/smart_ptr/shared_ptr.hpp>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <thread>

int main()
{
    using PointType = pcl::PointXYZ;

    auto box = pcl::PointCloud<PointType>::Ptr(new pcl::PointCloud<PointType>());

    for (int x = 0; x < 100; x++)
        for (int y = 0; y < 100; y++)
            for (int z = 0; z < 100; z++) {
            }

    auto view = boost::shared_ptr<pcl::visualization::PCLVisualizer>();
    view->setBackgroundColor(0, 0, 0);
    view->addPointCloud<PointType>(box, "box");

    view->spin();

    return 0;
}