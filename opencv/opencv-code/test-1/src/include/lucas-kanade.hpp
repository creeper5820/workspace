#pragma once

#include <opencv2/core.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/videoio.hpp>

#include <iostream>
#include <string>
#include <sys/types.h>
#include <vector>

inline cv::Mat roi_extract(cv::Mat inputFrame, std::vector<cv::Point> points)
{
    cv::Mat dst, src;
    src = inputFrame;
    cv::Mat ROI = cv::Mat::zeros(src.size(), CV_8UC1);
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Point> pts;

    for (int i = 0; i < points.size(); i++) {
        pts.push_back(points[i]);
    }

    contours.push_back(pts);
    cv::drawContours(ROI, contours, 0, cv::Scalar(255), -1);
    src.copyTo(dst, ROI);

    return dst;
}

inline void test_image_show()
{
    std::string path
        = "/workspaces/codespace/opencv/opencv-code/test-1/video/otuai.png";

    cv::Mat test_pic = cv::imread(path);

    cv::imshow("test-picture-show", test_pic);

    cv::waitKey();
}

inline void test_video_show(const std::string& file)
{
    // open the video
    cv::VideoCapture video(file);
    if (!video.isOpened()) {
        std::cerr << "unable to open the video" << std::endl;
    }

    cv::Mat video_show;

    while (1) {
        video >> video_show;

        if (video_show.empty())
            break;

        cv::imshow("test-video-show", video_show);

        if (cv::waitKey(10) == 'q')
            break;
    }
}

inline int lucas_kanade(const std::string& file, bool is_save = 0)
{
    // open the video
    cv::VideoCapture video(file);
    if (!video.isOpened()) {
        std::cerr << "unable to open the video" << std::endl;
        return -1;
    }

    // skip
    int frame_count = 0;
    cv::Mat video_trash;

    while (frame_count < 1300) {
        video >> video_trash;
        frame_count++;
    }

    // variables
    cv::Mat frame_old;
    cv::Mat frame_new;
    cv::Mat frame_gray_old;
    cv::Mat frame_gray_new;

    std::vector<cv::Point2f> points_0;
    std::vector<cv::Point2f> points_1;
    std::vector<cv::Point2f> points_good;

    std::vector<uchar> points_status;
    std::vector<float> points_errors;

    std::vector<cv::Scalar> point_colors;

    cv::Mat video_show;
    cv::Mat mask_show;

    uint frames_track;
    uint frames_track_max = 20;

    float roi_area[4][2] = {
        { 0, 1 }, { 0, 1 },
        { 1, 0 }, { 1, 0 }
    };

    // random colors
    cv::RNG colors_rng;

    for (int i = 0; i < 100; i++) {
        point_colors.push_back(cv::Scalar(
            colors_rng.uniform(0, 256),
            colors_rng.uniform(0, 256),
            colors_rng.uniform(0, 256)));
    }

    video >> frame_old;
    if (frame_old.empty()) {
        return -1;
    }

    std::vector<cv::Point> points;
    double max_X = frame_old.size().width;
    double max_Y = frame_old.size().height;
    points.push_back(cv::Point(max_X * 0.32, max_Y * 1.0)); // LD
    points.push_back(cv::Point(max_X * 1.0, max_Y * 1.0)); // RD
    points.push_back(cv::Point(max_X * 1.0, max_Y * 0.0)); // RU
    points.push_back(cv::Point(max_X * 0.17, max_Y * 0.0)); // LU

    frames_track = 0;

    while (1) {
        // get begining frame
        video >> frame_old;
        if (frame_old.empty()) {
            break;
        }
        frame_old = roi_extract(frame_old, points);

        // get feature points
        cv::cvtColor(frame_old, frame_gray_old, cv::COLOR_BGR2GRAY);
        cv::goodFeaturesToTrack(
            frame_gray_old, points_0,
            100, 0.3, 3, cv::Mat(), 3);

        // create new mask for drawing
        mask_show = cv::Mat::zeros(frame_old.size(), frame_old.type());

        while (frames_track < frames_track_max) {

            // read new frame
            video >> frame_new;
            if (frame_new.empty())
                break;

            frame_new = roi_extract(frame_new, points);

            // get new gray frame
            cv::cvtColor(frame_new, frame_gray_new, cv::COLOR_BGR2GRAY);

            // calculate optical flow
            cv::TermCriteria criteria = cv::TermCriteria(
                cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                10, 0.03);

            cv::calcOpticalFlowPyrLK(
                frame_gray_old, frame_gray_new,
                points_0, points_1,
                points_status, points_errors,
                cv::Size(15, 15), 2, criteria);

            // visualization
            for (int i = 0; i < points_0.size(); i++) {
                if (points_status[i] == 1) {

                    // select
                    points_good.push_back(points_1[i]);

                    // draw
                    cv::line(mask_show, points_1[i], points_0[i], point_colors[i], 2);
                    cv::circle(frame_new, points_1[i], 5, point_colors[i], -1);
                }
            }

            frame_gray_old = frame_gray_new.clone();
            points_0 = points_good;
            points_good.clear();

            cv::add(frame_new, mask_show, video_show);
            cv::imshow("test-flow", video_show);

            if (cv::waitKey() == 'q')
                return 1;

            frames_track++;
        }

        // empty the feature points
        points_0.clear();
        points_1.clear();
        points_good.clear();
        points_errors.clear();
        points_status.clear();

        frames_track = 0;
    }

    return 0;
}
