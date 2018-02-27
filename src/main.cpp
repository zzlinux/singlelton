#include <iostream>
#include "thread/ThreadController.h"
#include "calibration/RgbdCalibration.h"
#include "calibration/MonoCameraCalibration.h"

using namespace hitcrt;

enum appMode{
    RGBDCALIBRATION,
    MONOCAMERACALIBRATION,
    ROBOCON
};
int main() {
    std::cout << "Hello, World!" << std::endl;
    cv::FileStorage fs("../config/param.yaml",cv::FileStorage::READ);
    int mode = fs["appMode"];
    fs.release();
    switch (mode)
    {
        case RGBDCALIBRATION:
        {
            std::cout<<"RGBD CALIBRATION APP"<<std::endl;
            RgbdCalibration calibration;
            calibration.run();
            break;
        }
        case MONOCAMERACALIBRATION:
        {
            std::cout<<"MONOCAMERA CALIBRATION APP"<<std::endl;
            MonoCameraCalibration calibration;
            calibration.run();
            break;
        }
        case ROBOCON:
        {
            std::cout<<"ROBOCON APP"<<std::endl;
            ThreadController thread;
            thread.init();
            thread.run();
            break;
        }
        default:
            break;
    }
    return 0;
}