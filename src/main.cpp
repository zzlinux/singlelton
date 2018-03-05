#include <iostream>
#include "makeParam.h"
#include "ThreadController.h"
#include "CalibrationFactory.h"
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
            CalibrationProduct * calibration = CalibrationFactory::CreateCalibration("rgbdCalibration");
            calibration->run();
            break;
        }
        case RGBDRECORDER:
        {
            std::cout<<"RGBD RECORDER APP"<<std::endl;
            CalibrationProduct * rgbdrecorder = CalibrationFactory::CreateCalibration("rgbdRecorder");
            rgbdrecorder->run();
            break;
        }
        case MONOCAMERACALIBRATION:
        {
            std::cout<<"MONOCAMERA CALIBRATION APP"<<std::endl;
            CalibrationProduct * calibration = CalibrationFactory::CreateCalibration("monoCameraCalibration");
            calibration->run();
            break;
        }
#endif
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