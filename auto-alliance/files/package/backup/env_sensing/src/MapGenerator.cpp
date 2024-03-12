//
// Created by soulde on 2023/6/24.
//

#include "env_sensing/MapGenerator.h"

int MapGenerator::genMap() {
//    pcl::PointCloud<pcl::PointXYZ>::Ptr tmp(new pcl::PointCloud<pcl::PointXYZ>);
    outCloudMap->clear();
    ocTree->clear();
//    for (const auto &pc: buffer) {
    for (auto p: (*(buffer.end() - 1))->points) {
        double dis2 = p.x * p.x + p.y * p.y;
        if (dis2 < 25 && dis2 > 0.25
//                    && p.x > 0
//                                        && p.z > 0 && p.z < 4
                ) {
            ocTree->updateNode(p.x, p.y, 0.55 - p.z, true);
        }
    }
//    }
    ocTree->updateInnerOccupancy();

    grid_map::Position3 min_bound(-5, -5, -5);
    grid_map::Position3 max_bound(5, 5, 5);


    grid_map::GridMapOctomapConverter::fromOctomap(*ocTree, "elevation", map, &min_bound, &max_bound);

    const float minValue = map.get("elevation").minCoeffOfFinites();
    const float maxValue = map.get("elevation").maxCoeffOfFinites();


    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {

        if (!map.isValid(*iterator, basicLayer)) {
            grid_map::Position pos;
            map.getPosition(*iterator, pos);
            if (pos(0) * pos(0) + pos(1) * pos(1) < 0.25) {
                map.at(slopeLayer, *iterator) = 0;
            } else {
                map.at(slopeLayer, *iterator) = 0.1;
            }
        } else if (map.at(basicLayer, *iterator) < -0.2) {
            map.at(basicLayer, *iterator) *=-1;
        }
    }
    grid_map::GridMapCvConverter::toImage<float, 1>(map, basicLayer, CV_32FC1, 0, maxValue, cvMap);
//    cv::medianBlur(cvMap, cvMap, 3);
//    cv::imshow("cvMap", cvMap);
    cv::Mat dKernel5 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(5, 5));
    cv::Mat dKernel3 = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::dilate(cvMap, cvMap, dKernel5);
    cv::erode(cvMap, cvMap, dKernel5);
    cv::blur(cvMap, cvMap, cv::Size(3, 3));
//    cv::flip(cvMap, cvMap, 1);

    grid_map::GridMapCvConverter::addLayerFromImage<float_t, 1>(cvMap, basicLayer, map, 0, maxValue);
    cv::Mat slope, slopeU, slopex, slopey;

    float res = map.getResolution();
    cv::filter2D(cvMap, slopex, CV_32FC1, kernelX);
    cv::filter2D(cvMap, slopey, CV_32FC1, kernelY);

    cv::sqrt(slopex.mul(slopex) + slopey.mul(slopey), slope);
//    slope.convertTo(slopeU, CV_16UC1);
    for (int i = 0; i < slope.rows; i++) {
        auto p = slope.ptr<float>(i);
        for (int j = 0; j < slope.cols; j++) {
//            outCloudMap->push_back(
//                    pcl::PointXYZ(res * (cvMap.cols / 2 - i + pos(0)), res * (cvMap.rows / 2 - j + pos(1)), p[j]));
        }
    }
//    cv::dilate(slope, slope,dKernel3);
//    cv::imshow("slope", slope);
//    std::cout << cvMap.size << std::endl;
//    cv::Mat mask, inpainted;
//    grid_map::GridMapCvConverter::toImage<uint8_t, 1>(map, "inpaint_mask", CV_8UC1, mask);
//    cv::inpaint(cvMap, mask, inpainted, 3, cv::INPAINT_NS);
//    cv::threshold(slope, slope, 1.4, 1, CV_THRESH_BINARY);
    outCloudMap->clear();
    grid_map::GridMapCvConverter::addLayerFromImage<float_t, 1>(slope, slopeLayer, map, minValue, maxValue);

    for (grid_map::GridMapIterator iterator(map); !iterator.isPastEnd(); ++iterator) {
        grid_map::Position p;
        map.getPosition(*iterator, p);
        if (map.at(slopeLayer, *iterator) < 0.25) {
            map.at(slopeLayer, *iterator) = 0;
//            outCloudMap->push_back(pcl::PointXYZ(p(0), p(1), -1));
        } else {
            map.at(slopeLayer, *iterator) = 1;
            outCloudMap->push_back(pcl::PointXYZ(p(0), p(1), -1));
        }
    }
//    std::cout << inpainted << std::endl;
//    cv::imshow("src", cvMap);
//    cv::imshow("slope", slopeU);
//    cv::imshow("srcx", slopex);
//    cv::imshow("srcy", slopey);
//    cv::waitKey(1);


    return 0;
}

MapGenerator::MapGenerator(int bufSize) : outCloudMap(new pcl::PointCloud<pcl::PointXYZ>),
                                          ocTree(new octomap::OcTree(0.05)),
                                          _bufSize(bufSize),
                                          map(grid_map::GridMap({basicLayer, slopeLayer})),
                                          Ex(Eigen::Matrix<double, 4, 4>::Identity()) {
    Ex.block<3, 3>(0, 0) = Eigen::AngleAxisd(180, Eigen::Vector3d::UnitZ()).toRotationMatrix();

}
