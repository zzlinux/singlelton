//
// Created by robocon on 18-1-14.
//

#include "RadarController.h"
#include "../thread/Param.h"
namespace hitcrt
{
    void RadarController::run()
    {
        m_radarDataThread = boost::thread(boost::bind(&RadarController::m_radarReadFrame,this));
        m_radarProcessThread = boost::thread(boost::bind(&RadarController::m_radarProcess,this));
        m_radarDataThread.join();

    }
    void RadarController::m_radarReadFrame()
    {
        std::cout<<"radarDataThread "<<m_radarDataThread.get_id()<<std::endl;
        while(Param::m_process)
        {
            struct timeval st,en;
            long time_stamp = 0;
            gettimeofday(&st,NULL);
            {
                boost::unique_lock<boost::shared_mutex> writelock(radarlock);
                readData.clear();
                if (!urg.get_distance(readData, &time_stamp)) {
                    std::cout << "Urg_driver::get_distance(): " << urg.what() << std::endl;
                    throw -1;
                }
            }
            gettimeofday(&en,NULL);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            //std::cout<<"radar write time is "<<std::dec<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
        }
    }
    void RadarController::m_radarProcess()
    {
        std::cout<<"radarProcessThread "<<m_radarProcessThread.get_id()<<std::endl;

        std::vector<float> position;
        std::vector<float> sendflag(1);
        char mode;
        while(Param::m_process)
        {
            data.clear();
            {
                boost::shared_lock<boost::shared_mutex> readlock(radarlock);
                if(readData.empty())continue;
                data = readData;
            }
            if(Param::m_radarMode==0){sendflag[0] = 0;mode = Param::m_radarMode;}
            else if(Param::m_radarMode ==1){sendflag[0] = 1;mode = Param::m_radarMode;}
            else {sendflag[0] = 10;mode = Param::m_radarMode;}
            Param::serial->send(SerialApp::SEND_HEART_BEAT,sendflag);
            bool isLocationValued = apply(position,mode);
            if(isLocationValued==true)
            {
                if(Param::radarLocation.debug)recorder(position);
                Param::serial->send(SerialApp::SEND_RADAR,position);
            }
            position.clear();
        }
    }
    bool RadarController::apply(std::vector<float>&position,char mode)
    {
        struct timeval st,en;
        gettimeofday(&st,NULL);
        position.clear();
        std::vector<cv::Point2d> VecPoint;
        depth2xy(data,VecPoint,mode);
        std::vector<std::vector<cv::Point2d> >lines;
        MyRansac(VecPoint,lines);
        bool isLocationValued = getRadarPosition(lines,position,mode);
        if(isLocationValued) averageFilter(slidewindow,position);
        if(Param::radarLocation.debug) {
            showResultImg(position,lines,isLocationValued);
            myimshow(VecPoint);
        }
        gettimeofday(&en,NULL);
        //std::cout<<"radar process time is "<<std::dec<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
        return isLocationValued;
    }
    bool RadarController::getRadarPosition(std::vector<std::vector<cv::Point2d> >&lines,std::vector<float > &position,char mode)
    {
        //std::cout<<"getRadarPosition: lines.size = "<<lines.size()<<std::endl;
        if(lines.size()==0) return false;
        std::vector<float>tempL;
        if(mode ==0)
        {
            for(auto l:lines)
            {
                tempL.clear();
                getLocation(l,tempL);
                if(abs(tempL[1])>25){
                    //std::cout<<" angle too big "<<tempL[0]<<","<<tempL[1]<<std::endl;
                    continue;
                }
                position.assign(tempL.begin(),tempL.end());
            }
            if(position.empty())return false;
        }else if(mode ==1)
        {
            for(auto l:lines)
            {
                tempL.clear();
                getLocation(l,tempL);
                if(abs(tempL[1])<65)continue;
                position.assign(tempL.begin(),tempL.end());
            }
            if(position.empty())return false;
        }else return false;
        return true;
    }
    void RadarController::getLocation(std::vector<cv::Point2d>&line,std::vector<float> &position)
    {
        LINEKB kb;
        float x,angle;
        cv::Vec4f lparm;
        cv::fitLine(line,lparm,CV_DIST_HUBER,0,0.01,0.01);
        if(lparm[0]!=0)
        {
            kb.k = lparm[1]/lparm[0];
            kb.b = lparm[3]-kb.k*lparm[2];
            x = CalPointLineDistance(cv::Point2d(0,0),kb);
            angle = CalYawAngle(kb);
            position.push_back(x);
            position.push_back(angle);
        }else{
            x = abs(lparm[2]);
            angle = 0;
            position.push_back(x);
            position.push_back(angle);
        }
    }
}