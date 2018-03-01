//
// Created by robocon on 18-3-1.
//

#ifndef ROBOCON_RGBDRECORDER_H
#define ROBOCON_RGBDRECORDER_H

namespace hitcrt
{
    class RgbdRecorder {
    public:
        RgbdRecorder();
        ~RgbdRecorder(){};
        int run();

    private:
        char * fileurl;
    };
}


#endif //ROBOCON_RGBDRECORDER_H
