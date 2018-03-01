//
// Created by robocon on 18-1-28.
//

#include "ApriltagController.h"
#include "../thread/Param.h"
#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <sys/time.h>

using namespace std;
namespace hitcrt
{
    const char* windowName = "apriltags_demo";
    const double PI = 3.14159265358979323846;
    const double TWOPI = 2.0*PI;
    ApriltagController::ApriltagController() : m_tagDetector(NULL),
                                 m_tagCodes(AprilTags::tagCodes36h11),

                                 m_draw(true),
                                 m_timing(false),

                                 m_width(640),
                                 m_height(480),
                                 m_tagSize(0.166),
                                 m_fx(600),
                                 m_fy(600),
                                 m_px(m_width/2),
                                 m_py(m_height/2),

                                 m_exposure(-1),
                                 m_gain(-1),
                                 m_brightness(-1),
                                 m_deviceId(1)
    {
        m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

        // find and open a USB camera (built in laptop camera, web cam etc)
        m_cap = cv::VideoCapture(m_deviceId);
        if(!m_cap.isOpened()) {
            cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
            exit(1);
        }
        m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
        m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
        cout << "Camera successfully opened (ignore error messages above...)" << endl;
        cout << "Actual resolution: "
             << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
             << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

    }
    ApriltagController::~ApriltagController()
    {}
    inline double ApriltagController::standardRad(double t) {
        if (t >= 0.) {
            t = fmod(t+PI, TWOPI) - PI;
        } else {
            t = fmod(t-PI, -TWOPI) + PI;
        }
        return t;
    }
    void ApriltagController::wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
        yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
        double c = cos(yaw);
        double s = sin(yaw);
        pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
        roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
    }
    void ApriltagController::print_detection(AprilTags::TagDetection& detection) {
        cout << "  Id: " << detection.id
             << " (Hamming: " << detection.hammingDistance << ")";

        // recovering the relative pose of a tag:

        // NOTE: for this to be accurate, it is necessary to use the
        // actual camera parameters here as well as the actual tag size
        // (m_fx, m_fy, m_px, m_py, m_tagSize)

        Eigen::Vector3d translation;
        Eigen::Matrix3d rotation;
        detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                                 translation, rotation);

        Eigen::Matrix3d F;
        F <<
          1, 0,  0,
                0,  -1,  0,
                0,  0,  1;
        Eigen::Matrix3d fixed_rot = F*rotation;
        double yaw, pitch, roll;
        wRo_to_euler(fixed_rot, yaw, pitch, roll);

        cout << "  distance=" << translation.norm()
             << "m, x=" << translation(0)
             << ", y=" << translation(1)
             << ", z=" << translation(2)
             << ", yaw=" << yaw
             << ", pitch=" << pitch
             << ", roll=" << roll
             << endl;

        // Also note that for SLAM/multi-view application it is better to
        // use reprojection error of corner points, because the noise in
        // this relative pose is very non-Gaussian; see iSAM source code
        // for suitable factors.
    }
    void ApriltagController::readFrameFromCamera()
    {
        boost::unique_lock<boost::shared_mutex> writelock(cameralock);
        m_cap>>frame;
    }
    bool ApriltagController::getFrame()
    {
        boost::shared_lock<boost::shared_mutex> readlock(cameralock);
        if(frame.empty())return false;
        readFrame = frame.clone();
    }
    void ApriltagController::apply()
    {
        if(readFrame.empty())return;
        // detect April tags (requires a gray scale image)
        cv::Mat image_gray;
        cv::cvtColor(readFrame, image_gray, CV_BGR2GRAY);
        vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
        // print out each detection
        cout << detections.size() << " tags detected:" << endl;
        for (int i=0; i<detections.size(); i++) {
            print_detection(detections[i]);
        }
        // show the current image including any detections
        if (m_draw) {
            for (int i=0; i<detections.size(); i++) {
                // also highlight in the image
                detections[i].draw(readFrame);
            }
            if(Param::apriltag.debug)Param::mimshow(windowName, readFrame); // OpenCV call
        }
        // exit if any key is pressed
        if (cv::waitKey(1) >= 0) return;
    }
    void ApriltagController::m_apriltagReadFrame()
    {
        std::cout<<"apriltagDataThread id "<<m_apriltagDataThread.get_id()<<std::endl;
        while (Param::m_process)
        {
            struct timeval st,en;
            gettimeofday(&st,NULL);
            readFrameFromCamera();
            gettimeofday(&en,NULL);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            //std::cout<<"camera write time is "<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
        }
    }
    void ApriltagController::m_apriltagProcess()
    {
        std::cout <<"apriltagProcessThread id: "<<m_apriltagProcessThread.get_id()<<std::endl;
        while (Param::m_process)
        {
            struct timeval st,en;
            gettimeofday(&st,NULL);
            getFrame();
            apply();
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            //std::cout<<"camera write time is "<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
        }
    }
    void ApriltagController::run()
    {
        m_apriltagProcessThread = boost::thread(boost::bind(&ApriltagController::m_apriltagProcess,this));
        m_apriltagDataThread = boost::thread(boost::bind(&ApriltagController::m_apriltagReadFrame,this));
        m_apriltagDataThread.join();
    }
}

