//
// Created by robocon on 18-3-1.
//

#ifndef ROBOCON_RGBDRECORDER_H
#define ROBOCON_RGBDRECORDER_H

#include "CalibrationFactory.h"
namespace hitcrt
{
    class RgbdRecorder :public CalibrationProduct
    {
    public:
        RgbdRecorder();
        ~RgbdRecorder(){};
        void run();

    private:
        char * fileurl;
    };
}


#endif //ROBOCON_RGBDRECORDER_H
