//
// Created by robocon on 18-3-10.
//

#ifndef LIBFREENECT2_EXAMPLES_KINECT_H
#define LIBFREENECT2_EXAMPLES_KINECT_H

#include <opencv2/core/core.hpp>
#include <libfreenect2/libfreenect2.hpp>
#include <libfreenect2/frame_listener_impl.h>
#include <libfreenect2/registration.h>
#include <libfreenect2/packet_pipeline.h>

namespace hitcrt
{
    class KinectCamera {
    public:
        enum Mode{
            Live_mode,
            File_mode
        };
        KinectCamera();
        ~KinectCamera();
        cv::Mat getFrameDepth();
        cv::Mat getFrameRGB();
        static void registrate(cv::Mat &color,cv::Mat &dep,cv::Mat &registrate,cv::Mat &undistort);
        static void undistortDepth(cv::Mat depth,cv::Mat &depthUndistorted);
        static void registrate(cv::Mat &color,cv::Mat &dep,cv::Mat &registrate);
    private:
        bool enable_rgb = true;
        bool enable_depth = true;
        libfreenect2::Freenect2 freenect2;
        libfreenect2::Freenect2Device *dev = 0;
        int types = libfreenect2::Frame::Color | libfreenect2::Frame::Depth;
        libfreenect2::SyncMultiFrameListener listener;
        libfreenect2::FrameMap frames;
        static libfreenect2::Registration* registration;
        static libfreenect2::Frame undistorted, registered;
        libfreenect2::Frame *rgb,*depth;

        bool mirror = false;
        int rgbFlag = 2,depthFlag = 4,sumRgbDepth,frameFlag;
        void getFrames();
        void frameSync(int id);
    };


#endif //LIBFREENECT2_EXAMPLES_KINECT_H
}
