//
// Created by xuduo on 17-3-17.
//

#include "Param.h"
#include <opencv2/highgui/highgui.hpp>
namespace  hitcrt{
    bool Param::debug;
    Param::task Param::trace;
    Param::task Param::cameraLocation;
    Param::task Param::radarLocation;
    Param::task Param::apriltag;
    Param::info Param::traceinfo;


    float Param::FX = 366.534;  //Kinect 1号
    float Param::FY = 366.534;
    float Param::CX = 255.5;
    float Param::CY = 204.961;
    float Param::CAMERA_FACTOR = 1000.0;

    cv::Mat Param::RT01 = cv::Mat(4,4,CV_32FC1, cv::Scalar(0));
    cv::Mat Param::cameraLocationIntrinsic_0;
    cv::Mat Param::cameraLocationCoeffs_0;
    cv::Mat Param::cameraLocationIntrinsic_1;
    cv::Mat Param::cameraLocationCoeffs_1;
    pthread_mutex_t Param::mutex;
    void Param::mimshow(std::string winname, cv::Mat &mat)
    {
        pthread_mutex_lock(&mutex);
        cv::imshow(winname,mat);
        pthread_mutex_unlock(&mutex);
    }

    boost::atomic_char Param::m_radarMode;
    boost::atomic_char Param::m_traceMode;
    boost::atomic_char Param::m_throwArea;
    boost::atomic_bool Param::m_process;
    std::unique_ptr<SerialApp> Param::serial = std::unique_ptr<SerialApp>(new SerialApp("/dev/ttyUSB0",115200));


    Singleton* Singleton::m_instance = NULL;
    pthread_mutex_t Singleton::mutex;
    int Singleton::count = 1;
    Singleton* Singleton::getInstance()
    {
        if(m_instance == NULL)
        {
            pthread_mutex_lock(&mutex);
            if(m_instance == NULL)
            {
                m_instance = new Singleton();
            }
            pthread_mutex_unlock(&mutex);
        }
        return m_instance;
    }
    Singleton::Singleton()
    {
        std::cout<<"实例化了"<<count<<"个对象"<<std::endl;
        count++;

        FX = 366.534;  //Kinect 1号
        FY = 366.534;
        CX = 255.5;
        CY = 204.961;
        CAMERA_FACTOR = 1000.0;
        m_radarMode = 0;
        m_traceMode = 0;
        m_throwArea = 2;
        m_process = true;
        cv::FileStorage fs(cv::String("../config/param.yaml"), cv::FileStorage::READ);
        assert(fs.isOpened());
        cv::FileNode task = fs["TASK"],debug = fs["DEBUG"],traceN = fs["trace"],cameraLocationN = fs["cameraLocation"];
        cv::FileNode c = traceN[(std::string)traceN["ballcolor"]],g = traceN[(std::string)traceN["ballgold"]];
        trace = {(int)task["trace"],(int)debug["trace"]};
        cameraLocation = {(int)task["cameraLocation"],(int)debug["cameraLocation"]};
        radarLocation = {(int)task["radarLocation"],(int)debug["radarLocation"]};
        apriltag = {(int)task["apriltag"],(int)debug["apriltag"]};
        traceinfo = {(int)traceN["mode"],(std::string)traceN["oni"],
                            {{(int)c["hmin"],(int)c["hmax"]},{(int)c["smin"],(int)c["smax"]},{(int)c["vmin"],(int)c["vmax"]}},
                            {{(int)g["hmin"],(int)g["hmax"]},{(int)g["smin"],(int)g["smax"]},{(int)g["vmin"],(int)g["vmax"]}},
        };
        traceN["CIRCLE"]>>traceinfo.circle_range;
        traceN["CIRCLE2D"]>>traceinfo.circle_range2d;
        traceN["BALL"]>>traceinfo.ball_range;
        traceN["BALL2D"]>>traceinfo.ball_range2d;
        traceN["RT01"] >>RT01;
        cameraLocationN["OV2710_INTRINSIC_0"] >> Param::cameraLocationIntrinsic_0;
        cameraLocationN["OV2710_COEFFS_0"] >> Param::cameraLocationCoeffs_0;
        cameraLocationN["OV2710_INTRINSIC_1"] >> Param::cameraLocationIntrinsic_1;
        cameraLocationN["OV2710_COEFFS_1"] >> Param::cameraLocationCoeffs_1;
        fs.release();
    }
}
