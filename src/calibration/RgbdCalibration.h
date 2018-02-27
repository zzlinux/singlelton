//
// Created by robocon on 18-1-31.
//

#ifndef ROBOCON_RGBDCALIBRATION_H
#define ROBOCON_RGBDCALIBRATION_H

#include "rgbd_camera.h"
#include <memory>
namespace hitcrt
{
    class RgbdCalibration {
    public:
        RgbdCalibration();
        ~RgbdCalibration(){};
        void run();
    private:
        std::unique_ptr<RGBDcamera> cap;
        cv::Mat rgb,dep,depth8U;
        std::string ground = "../../grounds.txt";
        void computeRT01();
        void getGroundCloud();
    };
}


#endif //ROBOCON_RGBDCALIBRATION_H
