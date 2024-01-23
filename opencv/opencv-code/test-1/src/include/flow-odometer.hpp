#pragma once

#include <opencv2/videoio.hpp>

class FlowOdometer {
private:
    std::string name_file_;
    float distance_;

private:
    cv::Mat solution_perspection();

public:
};