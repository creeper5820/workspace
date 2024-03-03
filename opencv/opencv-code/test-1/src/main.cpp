#include "lucas-kanade.hpp"

#include <opencv2/core.hpp>
#include <opencv2/videoio.hpp>

// int main(int argc, char** argv)
// {
//     std::string path;
//     path = "/workspaces/codespace/opencv/opencv-code/test-1/video/test-2.mp4";

//     lucas_kanade(path);
// }

int main()
{
    auto capture = cv::VideoCapture();

    try {
        capture.open("22");
        capture.isOpened();

    } catch (cv::Exception& e) {

        const char* error = e.what();
        std::cout << "excepthon: " << error << std::endl;
    }
}