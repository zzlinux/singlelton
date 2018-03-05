//
// Created by robocon on 18-3-5.
//

#ifndef ROBOCON_CALIBRATIONFACTORY_H
#define ROBOCON_CALIBRATIONFACTORY_H

#include "iostream"

namespace hitcrt
{
    class CalibrationProduct{
    public:
        virtual void run()=0;
    };
    class CalibrationFactory{
    public:
        static CalibrationProduct *CreateCalibration(std::string task);
    };
}


#endif //ROBOCON_CALIBRATIONFACTORY_H
