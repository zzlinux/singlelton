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
#include <vector>

using namespace std;
namespace hitcrt
{
    boost::mutex TagTask::lastSenddMutex;
    int TagTask::lastSend = 10;
    //const char* windowName = "apriltags_demo";
    const double PI = 3.14159265358979323846;
    const double TWOPI = 2.0*PI;
    ApriltagController::~ApriltagController()
    {}
    void ApriltagController::run()
    {
        m_tag0 = new TagTask(0);
        m_tag1 = new TagTask(1);
        m_apriltag0 = boost::thread(boost::bind(&ApriltagController::m_task0,this));
        m_apriltag1 = boost::thread(boost::bind(&ApriltagController::m_task1,this));
        m_apriltag0.join();
    }
    void ApriltagController::m_task0()
    {
        std::cout<<"apriltagThread0 id "<<m_apriltag0.get_id()<<std::endl;
        m_tag0->run();
    }
    void ApriltagController::m_task1()
    {
        std::cout<<"apriltagThread1 id "<<m_apriltag1.get_id()<<std::endl;
        m_tag1->run();
    }
    TagTask::TagTask(int id) : m_tagDetector(NULL),
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
                               m_deviceId(id)
    {
        m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

        // find and open a USB camera (built in laptop camera, web cam etc)
        if(m_deviceId ==1)
        {
            m_cap = new hitcrt::myVideoCap("/dev/video1",640,480,60);
            windowName = "tag0";
        }else
        {
            windowName = "tag1";
            m_cap = new hitcrt::myVideoCap("/dev/video2",640,480,60);
        }
        /*
        m_cap = cv::VideoCapture(m_deviceId);
        double fps = m_cap.get(CV_CAP_PROP_FPS);
        double width = m_cap.get(CV_CAP_PROP_FRAME_WIDTH);
        double height = m_cap.get(CV_CAP_PROP_FRAME_HEIGHT);
        std::cout<<"fps "<<fps<<"     (x,y) "<<width<<","<<height<<std::endl;
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
             */

    }
    void TagTask::run()
    {
        m_apriltagDataThread = boost::thread(boost::bind(&TagTask::m_apriltagReadFrame,this));
        m_apriltagProcessThread = boost::thread(boost::bind(&TagTask::m_apriltagProcess,this));
        m_apriltagDataThread.join();
    }
    void TagTask::m_apriltagReadFrame()
    {
        std::cout<<"apriltagDataThread id "<<m_apriltagDataThread.get_id()<<std::endl;
        while (Param::m_process)
        {
            struct timeval st,en;
            gettimeofday(&st,NULL);
            {
                boost::unique_lock<boost::shared_mutex> writelock(cameralock);
                m_cap->VideoGrab(frame);
            }
            gettimeofday(&en,NULL);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            std::cout<<"camera write time is "<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
        }
    }
    void TagTask::m_apriltagProcess()
    {
        std::cout <<"apriltagProcessThread id: "<<m_apriltagProcessThread.get_id()<<std::endl;
        while (Param::m_process)
        {
            struct timeval st,en;
            gettimeofday(&st,NULL);
            {
                boost::shared_lock<boost::shared_mutex> readlock(cameralock);
                if(frame.empty()) continue;
                readFrame = frame.clone();
            }
            gettimeofday(&en,NULL);
            apply();
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            std::cout<<"camera process time is "<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
        }
    }
    double TagTask::standardRad(double t) {
        if (t >= 0.) {
            t = fmod(t+PI, TWOPI) - PI;
        } else {
            t = fmod(t-PI, -TWOPI) + PI;
        }
        return t;
    }
    void TagTask::wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
        yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
        double c = cos(yaw);
        double s = sin(yaw);
        pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
        roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
    }
    void TagTask::print_detection(AprilTags::TagDetection& detection) {
        //cout << "  Id: " << detection.id
        //     << " (Hamming: " << detection.hammingDistance << ")";

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
        //cout << "  distance=" << translation.norm()
        //     << "m, x=" << translation(0)
        //     << ", y=" << translation(1)
        //     << ", z=" << translation(2)
        //     << ", yaw=" << yaw
        //     << ", pitch=" << pitch
        //cout     << ", roll=" << roll
        //     << endl;
        std::stringstream text;
        text << "Id: "<<detection.id
             << "    yaw: "<<yaw
             << "    pitch: "<<pitch
             << "    roll: "<<roll;
        message.lastId = message.id;
        if(yaw>1.3) message.id = 1;
        else if(yaw <-1.3) message.id = 0;
        else message.id = 10;
        if(message.lastId == message.id) message.count++;
        else message.count = 1;
        cv::putText(readFrame,text.str(),cv::Point(10,30),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(10,40,255),2);
    }
    void TagTask::apply()
    {
        if(readFrame.empty())return;
        cv::Mat image_gray;
        std::vector<cv::Mat> channels;
        cv::split(readFrame,channels);
        //cv::cvtColor(readFrame, image_gray, CV_BGR2GRAY);
        image_gray = channels[1];
        //cv::imshow("apriltag",image_gray);
        vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
        for (int i=0; i<detections.size(); i++) {
            print_detection(detections[i]);
        }
        if(message.count ==1)
        {
            lastSenddMutex.lock();
            if(lastSend != message.id)
                isSend = true;
            lastSend = message.id;
            lastSenddMutex.unlock();
            if(isSend)
            {
                Param::serial->send(SerialApp::SEND_APRILTAG,std::vector<float>(1,message.id));
                isSend = false;
            }
            std::stringstream text;
            text<<"message id  :  "<<message.id;
            cv::putText(readFrame,text.str(),cv::Point(10,80),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(0,255,25),2);
            message.lastId = 10;
            message.id = 10;
            message.count = 0;
        }
        if (Param::apriltag.debug) {
            for (int i=0; i<detections.size(); i++) {
                detections[i].draw(readFrame);
            }
            Param::mimshow(windowName, readFrame); // OpenCV call
            cv::waitKey(1);
        }
    }
}

