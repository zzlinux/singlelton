//
// Created by robocon on 18-1-9.
//

#include <opencv2/core/core.hpp>
#include "ThreadController.h"
#include "Param.h"

namespace hitcrt
{
    void ThreadController::init()
    {
        Param::m_radarMode = 0;
        Param::m_traceMode = 0;
        Param::m_throwArea = 2;
        Param::m_process = true;
        cv::FileStorage fs(cv::String("../config/param.yaml"), cv::FileStorage::READ);
        assert(fs.isOpened());
        cv::FileNode task = fs["TASK"],debug = fs["DEBUG"],trace = fs["trace"],cameraLocation = fs["cameraLocation"];
        cv::FileNode c = trace[(std::string)trace["ballcolor"]],g = trace[(std::string)trace["ballgold"]];
        Param::debug = (int)debug["master"];
        Param::trace = {(int)task["trace"],(int)debug["trace"]&&Param::debug};
        Param::cameraLocation = {(int)task["cameraLocation"],(int)debug["cameraLocation"]&&Param::debug};
        Param::radarLocation = {(int)task["radarLocation"],(int)debug["radarLocation"]&&Param::debug};
        Param::apriltag = {(int)task["apriltag"],(int)debug["apriltag"]&&Param::debug};
        Param::traceinfo = {(int)trace["mode"],(std::string)trace["oni"],
                        {{(int)c["hmin"],(int)c["hmax"]},{(int)c["smin"],(int)c["smax"]},{(int)c["vmin"],(int)c["vmax"]}},
                        {{(int)g["hmin"],(int)g["hmax"]},{(int)g["smin"],(int)g["smax"]},{(int)g["vmin"],(int)g["vmax"]}},
        };
        trace["CIRCLE"]>>Param::traceinfo.circle_range;
        trace["CIRCLE2D"]>>Param::traceinfo.circle_range2d;
        trace["BALL"]>>Param::traceinfo.ball_range;
        trace["BALL2D"]>>Param::traceinfo.ball_range2d;
        trace["RT01"] >> Param::RT01;
        cameraLocation["OV2710_INTRINSIC"] >> Param::cameraLocationIntrinsic;
        cameraLocation["OV2710_COEFFS"] >> Param::cameraLocationCoeffs;
        fs.release();
    }
    void ThreadController::run()
    {
        if(Param::radarLocation.start)
        {
            radarLocation = std::unique_ptr<TaskProduct>(TaskFactory::CreateTask("radarLocation"));
            m_radarLocationThread = boost::thread(boost::bind(&ThreadController::m_radarLocation,this));
        }
        if(Param::trace.start)
        {
            trace = std::unique_ptr<TaskProduct>(TaskFactory::CreateTask("trace"));
            m_traceThread = boost::thread(boost::bind(&ThreadController::m_trace,this));
        }
        if(Param::cameraLocation.start)
        {
            cameraLocation0 = std::unique_ptr<TaskProduct>(TaskFactory::CreateTask("cameraLocation",0));
            m_cameraLocation0Thread = boost::thread(boost::bind(&ThreadController::m_cameraLocation0,this));
            cameraLocation1 = std::unique_ptr<TaskProduct>(TaskFactory::CreateTask("cameraLocation",1));
            m_cameraLocation1Thread = boost::thread(boost::bind(&ThreadController::m_cameraLocation1,this));
        }
        if(Param::apriltag.start)
        {
            aprilTag = std::unique_ptr<TaskProduct>(TaskFactory::CreateTask("apriltag",1));
            m_apriltagThread = boost::thread(boost::bind(&ThreadController::m_apriltag,this));
        }
        m_communicationThread = boost::thread(boost::bind(&ThreadController::m_communication,this));

        if(Param::debug)
        {
            m_mutualThread = boost::thread(boost::bind(&ThreadController::m_mutual,this));
            m_mutualThread.join();
        }else m_communicationThread.join();
    }
    void ThreadController::m_communication()
    {
        std::cout<<"communicationThread id "<<m_communicationThread.get_id()<<std::endl;
        SerialApp::RECEIVE_FLAG flag;
        std::vector<float> data;
        while (Param::m_process)
        {
            boost::this_thread::interruption_point();
            Param::serial->receive(flag,data);
            std::cout<<"receive,flag"<< static_cast<int>(flag)<<std::endl;
            if(flag == SerialApp::RECEIVE_RADAR)
            {
                if(std::round(data[0]) == 0)            //horizontal
                    Param::m_radarMode = 0;
                else if(std::round(data[0]) == 1)       //vertical
                    Param::m_radarMode = 1;
            }
            else if(flag == SerialApp::RECEIVE_TRACE)
            {
                if(std::round(data[0]) == 1)            //throw area1
                    Param::m_throwArea = 1;
                else if(std::round(data[0]) == 2)       //throw area2
                    Param::m_throwArea = 2;
                else if(std::round(data[0] == 3))       //throw area3
                    Param::m_throwArea = 3;
                if(std::round(data[1]) == 0)
                    Param::m_traceMode = 0;
                else if(std::round(data[1]) == 1)       //throw first
                    Param::m_traceMode = 1;
                else if(std::round(data[1]) == 2)       //throw again
                    Param::m_traceMode = 2;
                std::cout<<"traceMode,area "<< static_cast<int>(Param::m_traceMode)<<","<< static_cast<int>(Param::m_throwArea)<<std::endl;
            }
        }
    }
    void ThreadController::m_mutual()
    {
        std::cout<<"mutualThread id "<<m_mutualThread.get_id()<<std::endl;
        while(true)
        {
            boost::this_thread::interruption_point();
            char ch = getchar();
            if(ch == 'q')
            {
                Param::m_process = false;
                break;
            }else if(ch == '0'){
                Param::m_traceMode = 0;
            }else if(ch == '1'){
                Param::m_traceMode = 1;
            }else if(ch == '2'){
                Param::m_traceMode = 2;
            }else if(ch == 'v'){        //vertical
                Param::m_radarMode = 1;
            }else if(ch == 'h'){        //horizontal
                Param::m_radarMode = 0;
            }
        }
    }

    void ThreadController::m_radarLocation()
    {
        std::cout <<"radarProcessThread id: "<<m_radarLocationThread.get_id()<<std::endl;
        if(radarLocation == NULL)
            radarLocation->error();
        else
            radarLocation->run();
    }
    void ThreadController::m_trace()
    {
        std::cout<<"traceThread id "<<m_traceThread.get_id()<<std::endl;
        if(trace == NULL)
            radarLocation->error();
        else
            trace->run();
    }
    void ThreadController::m_cameraLocation0()
    {
        std::cout<<"cameraLocation0Thread id "<<m_cameraLocation0Thread.get_id()<<std::endl;
        if(cameraLocation0 == NULL)
            radarLocation->error();
        else
            cameraLocation0->run();
    }
    void ThreadController::m_cameraLocation1()
    {
        std::cout<<"cameraLocation1Thread id "<<m_cameraLocation1Thread.get_id()<<std::endl;
        if(cameraLocation1 == NULL)
            cameraLocation1->error();
        else
            cameraLocation1->run();
    }
    void ThreadController::m_apriltag()
    {
        std::cout <<"apriltagThread id: "<<m_apriltagThread.get_id()<<std::endl;
        if(aprilTag == NULL)
            aprilTag->error();
        else
            aprilTag->run();
    }
}
