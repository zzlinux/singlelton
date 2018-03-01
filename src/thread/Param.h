//
// Created by xuduo on 17-3-17.
//

#ifndef VISIONCLOSUREV2_PARAM_H
#define VISIONCLOSUREV2_PARAM_H

#include <boost/thread.hpp>
#include <opencv2/opencv.hpp>

#include "serialapp.h"

namespace  hitcrt{
    struct Param {

        static float FX;
        static float FY;
        static float CX;
        static float CY;
        static float CAMERA_FACTOR;

        static cv::Mat RT01;
        static cv::Mat cameraLocationIntrinsic;
        static cv::Mat cameraLocationCoeffs;

        struct task{bool start;bool debug;};
        static task trace,cameraLocation,radarLocation,apriltag;
        static bool debug;

        struct colorhsv{
            struct {int min;int max;}h,s,v;
        };
        struct info{bool rgbdMode;
            std::string file;
            colorhsv cball;
            colorhsv gball;
            cv::Mat circle_range;
            cv::Mat circle_range2d;
            cv::Mat ball_range;
            cv::Mat ball_range2d;
        };
        static info traceinfo;

        static pthread_mutex_t mutex;
        static void mimshow(std::string winname, cv::Mat &mat);



        static boost::atomic_char m_radarMode;
        static boost::atomic_char m_traceMode;
        static boost::atomic_char m_throwArea;
        static boost::atomic_bool m_process;
        static std::unique_ptr<SerialApp> serial;
    };

    class Singleton
    {
    private:
        static Singleton* m_instance;
        Singleton();
        static pthread_mutex_t mutex;
        boost::mutex imlock;
        static int count;
    public:
        static Singleton* getInstance();
        void threadimshow(std::string winname, cv::Mat &mat);
        float FX;
        float FY;
        float CX;
        float CY;
        float CAMERA_FACTOR;
        cv::Mat RT01;

        cv::Mat cameraLocationIntrinsic;
        cv::Mat cameraLocationCoeffs;

        struct task{bool start;bool debug;};
        task trace,cameraLocation,radarLocation,apriltag;

        struct colorhsv{
            struct {int min;int max;}h,s,v;
        };
        struct info{bool rgbdMode;
            std::string file;
            colorhsv cball;
            colorhsv gball;
            cv::Mat circle_range;
            cv::Mat circle_range2d;
            cv::Mat ball_range;
            cv::Mat ball_range2d;
        };
        info traceinfo;

        boost::atomic_char m_radarMode;
        boost::atomic_char m_traceMode;
        boost::atomic_char m_throwArea;
        boost::atomic_bool m_process;
        std::unique_ptr<SerialApp> serial;
    };
}



#endif //VISIONCLOSUREV2_PARAM_H
