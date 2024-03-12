//
// Created by soulde on 2023/6/24.
//

#ifndef ENV_SENSING_MAPGENERATOR_H
#define ENV_SENSING_MAPGENERATOR_H

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <octomap/OcTree.h>
#include <grid_map_core/grid_map_core.hpp>
#include <grid_map_octomap/grid_map_octomap.hpp>
#include <grid_map_cv/GridMapCvConverter.hpp>

#include <opencv2/opencv.hpp>


class MapGenerator {
public:

    explicit MapGenerator(int bufSize);

    int genMap();

    void inputPointCloud(const pcl::PointCloud<pcl::PointXYZ>::Ptr &pc) {
//        pcl::transformPointCloud(*pc, *pc, Ex);
        buffer.push_back(pc);
        while (buffer.size() > _bufSize) {
            buffer.erase(buffer.begin());
        }
    }

    grid_map::GridMap &getMap() {
        return map;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr &getCloud() {
        return outCloudMap;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr &getDebugCloud() {
        return buffer[0];
    }

private:
    std::string basicLayer = "elevation", slopeLayer = "slope";
    pcl::PointCloud<pcl::PointXYZ>::Ptr outCloudMap;
    std::shared_ptr<octomap::OcTree> ocTree;
    grid_map::GridMap map;
    cv::Mat cvMap;

    const cv::Mat kernelX = (cv::Mat_<double>(3, 3) << 1, 0, -1, 1, 0, -1, 1, 0, -1);
    const cv::Mat kernelY = (cv::Mat_<double>(3, 3) << 1, 1, 1, 0, 0, 0, -1, -1, -1);
    Eigen::Matrix<double, 4, 4> Ex;
    float wallHeight = 3.;
    int _bufSize;
    std::vector<pcl::PointCloud<pcl::PointXYZ>::Ptr> buffer;
};


#endif //ENV_SENSING_MAPGENERATOR_H
