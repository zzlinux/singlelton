//
// Created by robocon on 18-1-14.
//

#include "RadarController.h"
#include "../thread/Param.h"
namespace hitcrt
{
    void RadarController::run()
    {
        std::vector<float> position;
        bool isLocationValued;          //判断激光雷达获得坐标是否正确
        std::vector<float> sendflag(1);
        char mode;
        while(Param::m_process)
        {
            if(Param::m_radarMode==0){sendflag[0] = 0;mode = Param::m_radarMode;}
            else if(Param::m_radarMode ==1){sendflag[0] = 1;mode = Param::m_radarMode;}
            else {sendflag[0] = 10;mode = Param::m_radarMode;}
            Param::serial->send(SerialApp::SEND_HEART_BEAT,sendflag);
            bool isLocationValued = apply(position,mode);
            if(isLocationValued==true)
            {
                recorder(position);
                Param::serial->send(SerialApp::SEND_RADAR,position);
            }
            position.clear();
        }
    }
    bool RadarController::apply(std::vector<float>&position,char mode)
    {
        position.clear();
        long time_stamp = 0;
        std::vector<long> data;
        if (!urg.get_distance(data, &time_stamp)) {
            std::cout << "Urg_driver::get_distance(): " << urg.what() << std::endl;
            return false;
        }
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