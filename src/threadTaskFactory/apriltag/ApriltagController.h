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
#include "TaskFactory.h"
#include "myVideoCap.h"
using namespace std;
namespace hitcrt
{
    class TagTask
    {
    public:
        TagTask(int id);
        ~TagTask(){};
        void run();
    private:
        char *windowName;
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

        //cv::VideoCapture m_cap;
        hitcrt::myVideoCap * m_cap;

        int m_exposure;
        int m_gain;
        int m_brightness;

        cv::Mat frame,readFrame;
        boost::shared_mutex cameralock;

        double standardRad(double t);
        void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);
        void print_detection(AprilTags::TagDetection& detection);
        void apply();
        boost::thread m_apriltagDataThread;
        boost::thread m_apriltagProcessThread;
        void m_apriltagReadFrame();
        void m_apriltagProcess();
        struct {int id = 10;int lastId = 10;int count = 0;} message;
        bool isSend = false;
        static int lastSend = 10;
        static boost::mutex lastSenddMutex;
    };
    class ApriltagController :public TaskProduct
    {
    public:
        ApriltagController(int id){};
        ~ApriltagController();
        void run();
    private:
        TagTask *m_tag0;
        TagTask *m_tag1;
        boost::thread m_apriltag0;
        boost::thread m_apriltag1;
        void m_task0();
        void m_task1();
    };
}
#endif //ROBOCON_APRILTAGCONTROLLER_H
