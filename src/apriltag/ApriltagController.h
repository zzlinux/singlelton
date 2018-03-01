//
// Created by robocon on 18-1-28.
//

#ifndef ROBOCON_APRILTAGCONTROLLER_H
#define ROBOCON_APRILTAGCONTROLLER_H

#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <boost/thread.hpp>

#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#include "lib/TagDetector.h"
#include "lib/Tag16h5.h"
#include "lib/Tag25h7.h"
#include "lib/Tag25h9.h"
#include "lib/Tag36h9.h"
#include "lib/Tag36h11.h"
#include <cmath>
#include "../thread/Param.h"
using namespace std;
namespace hitcrt
{
    class ApriltagController {
    public:
        AprilTags::TagDetector* m_tagDetector;
        AprilTags::TagCodes m_tagCodes;

        bool m_draw; // draw image and April tag detections?
        bool m_timing; // print timing information for each tag extraction call

        int m_width; // image size in pixels
        int m_height;
        double m_tagSize; // April tag side length in meters of square black frame
        double m_fx; // camera focal length in pixels
        double m_fy;
        double m_px; // camera principal point
        double m_py;

        int m_deviceId; // camera id (in case of multiple cameras)

        list<string> m_imgNames;

        cv::VideoCapture m_cap;

        int m_exposure;
        int m_gain;
        int m_brightness;

        cv::Mat frame,readFrame;
        boost::shared_mutex cameralock;

        ApriltagController(int id);
        ~ApriltagController();
        double tic();
        inline double standardRad(double t);
        void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);
        void readFrameFromCamera();
        bool getFrame();
        void print_detection(AprilTags::TagDetection& detection);
        void apply();
        boost::thread m_apriltagDataThread;
        boost::thread m_apriltagProcessThread;
        void m_apriltagReadFrame();
        void m_apriltagProcess();
        void run();
    };
}
#endif //ROBOCON_APRILTAGCONTROLLER_H
