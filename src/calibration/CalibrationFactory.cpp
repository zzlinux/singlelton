//
// Created by robocon on 18-3-5.
//

#include "CalibrationFactory.h"
#include "RgbdCalibration.h"
#include "MonoCameraCalibration.h"
#include "RgbdRecorder.h"

namespace hitcrt
{
    CalibrationProduct * CalibrationFactory::CreateCalibration(std::string task)
    {
        CalibrationProduct * p;
        if(task == "rgbdCalibration")
            p = new RgbdCalibration();
        else if(task == "monoCameraCalibration")
            p = new MonoCameraCalibration();
        else if(task == "rgbdRecorder")
            p = new RgbdRecorder();
        else p = NULL;
        return p;
    }
}