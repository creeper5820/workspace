#include <chrono>
#include <thread>

#include <eigen3/Eigen/Core>

#include "Control/Gimbal/GimbalInfantry.h"
#include "Core/Transformer/Tree.h"
#include "Util/FPSCounter/FPSCounter.h"
#include "Util/ROS/Node.h"

int main(int argc, char *argv[]) {
    std::thread thread(ros_util::init, argc, argv);
    thread.detach();
    
    for (int restartTime = 0;; ++restartTime) {
        LOG(INFO) << "Program started.";
        try {
            if (parameters::GimbalType == GimbalType::Infantry) {
                GimbalInfantry gimbal;
                gimbal.Always();
            } else if (parameters::GimbalType == GimbalType::Uav) {

            } else if (parameters::GimbalType == GimbalType::Sentry) {

            }
        }
        catch (const std::exception &e) {
            LOG(ERROR) << "Uncaught " << typeid(e).name() << ": " << e.what();
        }
        catch (const char *str) {
            LOG(ERROR) << "Uncaught const char* : " << str;
        }
        catch (...) {
            LOG(ERROR) << "Uncaught unknown error";
        }
        LOG(ERROR) << "Program crashed, will restart in 5 seconds... ";
        std::this_thread::sleep_for(std::chrono::seconds(5));
    }
    return 0;
}
