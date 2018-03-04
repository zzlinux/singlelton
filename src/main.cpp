#include <iostream>
#include "makeParam.h"
#include "thread/ThreadController.h"
#include "calibration/RgbdCalibration.h"
#include "calibration/MonoCameraCalibration.h"
#include "calibration/RgbdRecorder.h"

using namespace hitcrt;

enum appMode{
    RGBDCALIBRATION,
    RGBDRECORDER,
    MONOCAMERACALIBRATION,
    ROBOCON
};
int main() {
    std::cout << "Program will start, please hold on !"<<std::endl;
    cv::FileStorage fs("../config/param.yaml",cv::FileStorage::READ);
    int mode = fs["appMode"];
    int sleeptime = fs["sleeptime"];
    fs.release();
    sleep(sleeptime);
    std::cout << "Hello, World!" << std::endl;
    switch (mode)
    {
#if ROBOT == 1
        case RGBDCALIBRATION:
        {
            std::cout<<"RGBD CALIBRATION APP"<<std::endl;
            RgbdCalibration calibration;
            calibration.run();
            break;
        }
        case RGBDRECORDER:
        {
            std::cout<<"RGBD RECORDER APP"<<std::endl;
            RgbdRecorder rgbdrecorder;
            rgbdrecorder.run();
            break;
        }
        case MONOCAMERACALIBRATION:
        {
            std::cout<<"MONOCAMERA CALIBRATION APP"<<std::endl;
            MonoCameraCalibration calibration;
            calibration.run();
            break;
        }
#else
        case ROBOCON:
        {
            std::cout<<"ROBOCON APP"<<std::endl;
            ThreadController thread;
            thread.init();
            thread.run();
            break;
        }
#endif
        default:
            break;
    }
    return 0;
}