//
// Created by robocon on 18-2-26.
//
#include <pcl/visualization/cloud_viewer.h>
#include "TraceController.h"
#include "../thread/Param.h"
#include "BallAssociate.h"
#include "BallDetector.h"
#include "CircleDetector.h"
#include "FitTrace.h"
#include "Trajectory.h"
#include "transformer.h"
#include "Recorder.h"
namespace hitcrt
{
    void TraceController::run()
    {
        cap = std::unique_ptr<KinectCamera>(new KinectCamera);
        m_traceDataThread = boost::thread(boost::bind(&TraceController::m_traceReadFrame,this));
        m_traceProcessThread = boost::thread(boost::bind(&TraceController::m_traceProcess,this));
        m_traceProcessThread.join();
    }
    void TraceController::m_traceReadFrame()
    {
        std::cout<<"traceDataThread id "<<m_traceDataThread.get_id()<<std::endl;
        while (Param::m_process)
        {
            struct timeval st,en;
            gettimeofday(&st,NULL);
            boost::this_thread::sleep(boost::posix_time::milliseconds(40));
            cv::Mat rgb,dep;
            rgb = cap->getFrameRGB();
            dep = cap->getFrameDepth();
            {
                boost::unique_lock<boost::shared_mutex> writelock(kinectlock);
                colorFrameBuff.push_back(rgb);
                depthFrameBuff.push_back(dep);
                if(colorFrameBuff.size()>=50)
                {
                    std::cout<<"buff is enriched"<<std::endl;
                    colorFrameBuff.pop_front();
                    depthFrameBuff.pop_front();
                }
            }
            gettimeofday(&en,NULL);
            std::cout<<"kinect write time is "<<std::dec<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
        }
    }
    void TraceController::m_traceProcess()
    {
        std::cout<<"traceprocessThread id  "<<m_traceProcessThread.get_id()<<std::endl;
        double throwTimeStart;
        const int MAXTHROWTIME = 4;
        //pcl::visualization::CloudViewer view("cloud");
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        std::vector<pcl::PointXYZ> targets;
        CircleDetector circle;
        BallDetector ball;
        BallAssociate associate;
        Recorder recorder;
        char isHit = 10;
        while (Param::m_process)
        {
            cv::Mat color,depth;
            {
                boost::shared_lock<boost::shared_mutex> readLock(kinectlock);
                if (!depthFrameBuff.empty()) {
                    color = colorFrameBuff.front().clone();
                    depth = depthFrameBuff.front().clone();
                    colorFrameBuff.pop_front();
                    depthFrameBuff.pop_front();
                }
            }
            if(color.empty()||depth.empty()){
                //std::cout<<"empty"<<std::endl;
                continue;
            }
            struct timeval st,en;
            gettimeofday(&st,NULL);
            if(Param::m_traceMode==1){
                Param::m_traceMode = 2;
                if(Param::m_throwArea == 0)continue;
                if(!circle.detector(depth,cloud,Param::m_throwArea)){std::cout<<"find circle failed"<<std::endl;continue;}
                std::cout<<"find circle ok"<<std::endl;
            }else if(Param::m_traceMode ==2){
                throwTimeStart = cv::getTickCount();
                isHit = 10;
                ball.init(Param::m_throwArea);
                associate.init(Param::m_throwArea);
                Param::m_traceMode = 3;
            }else if(Param::m_traceMode ==3){
                targets.clear();
                ball.detector(color,depth,cloud,targets);
                /*std::vector<Trajectory> ballTraces;
                ballTraces.clear();
                associate.apply(color,targets,ballTraces);
                if(!ballTraces.empty())
                    std::cout<<"get traces: "<<static_cast<int>(ballTraces.size())<<std::endl;
                for(auto t:ballTraces)
                {
                    cv::Mat R21,R31;
                    FitTrace::fitting(t,R21,R31);
                    pcl::PointXYZ p;
                    p.y = circle.center3d.y;
                    p.x = (p.y-R21.at<float>(0))/R21.at<float>(1);
                    p.z = R31.at<float>(0)+R31.at<float>(1)*p.y+R31.at<float>(2)*p.y*p.y;
                    float distCen = sqrt((p.x-circle.center3d.x)*(p.x-circle.center3d.x)+(p.z-circle.center3d.z)*(p.z-circle.center3d.z));
                    cv::Point cen;
                    Transformer::invTrans(p,cen);
                    cv::circle(color,cen,3,cv::Scalar(200,30,58),-1);
                    if(distCen<0.4) {isHit = 1;std::cout<<"yesyesyesyesyesyesyesyesyesyesyes"<<std::endl;}
                    else {isHit = 0;std::cout<<"errorerrorerrorerrorerrorerrorerrorerror"<<std::endl;}
                    recorder.trace(p,circle.center3d,t.points,distCen,isHit);
                }
                double tracetime= ((double)cv::getTickCount() - throwTimeStart)/cv::getTickFrequency();
                std::vector<float> throwresult(1);
                if(isHit==1){
                    throwresult[0] = 1;
                    Param::serial->send(SerialApp::SEND_TRACE,throwresult);
                    Param::m_traceMode = 0;
                    //m_throwArea = 0;
                    //circle.isValued = false;
                    std::cout<<"yes tracetime: "<<tracetime<<std::endl;
                    continue;
                }else if(isHit==0) {
                    throwresult[0] = 0;
                    Param::serial->send(SerialApp::SEND_TRACE,throwresult);
                    Param::m_traceMode = 0;
                    //m_throwArea = 0;
                    //circle.isValued = false;
                    std::cout<<"failed tracetime: "<<tracetime<<std::endl;
                    continue;
                }
                if(tracetime>=MAXTHROWTIME){
                    throwresult[0] = 0;
                    Param::serial->send(SerialApp::SEND_TRACE,throwresult);
                    //circle.isValued = false;
                    Param::m_traceMode = 0;
                    //m_throwArea = 0;
                    std::cout<<"failed for tracetime: "<<tracetime<<std::endl;
                    continue;
                }
                 */
            }
            /***************************DEBUG VIEW*************************/
            if(!Param::trace.debug)continue;
            if(circle.isValued)
            {
                cv::circle(color,circle.ground2d,1,cv::Scalar(10,200,70),-1);
                cv::circle(color,circle.center2d,3,cv::Scalar(80,35,176),-1);
                cv::circle(color,circle.center2d,circle.radius2d,cv::Scalar(255,255,255),1);
                cv::circle(color,circle.center2d,circle.radius2dOut,cv::Scalar(255,255,255),1);
            }
            cv::Mat registed,undistort;
            KinectCamera::registrate(color,depth,registed,undistort);
            cv::flip(registed,registed,1);
            cv::Point ballCenter;
            for (auto p:targets) {
                Transformer::invTrans(p, ballCenter);
                cv::circle(registed, ballCenter, 20, cv::Scalar(255, 255, 255), 1);
            }
            Param::mimshow("color",registed);
            //Param::mimshow("depth",undistort);
            //view.showCloud(cloud);
            cloud->points.clear();
            cv::waitKey(5);
            gettimeofday(&en,NULL);
            std::cout<<"kinect process time is "<<std::dec<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
            /**************************DEBUG VIEW**************************/
        }
    }
}
