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
        while(true)
        {
            boost::this_thread::interruption_point();
            if(Param::m_radarMode==0){sendflag[0] = 0;mode = Param::m_radarMode;}
            else if(Param::m_radarMode ==1){sendflag[0] = 1;mode = Param::m_radarMode;}
            else {sendflag[0] = 10;mode = Param::m_radarMode;}
            Param::serial->send(SerialApp::SEND_HEART_BEAT,sendflag);
            bool isLocationValued = apply(position,mode);
            if(isLocationValued==true)
            {
                //std::cout<<"yesyesyesyesyesyesyesyesyesyes"<<std::endl;
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
        //myimshow(VecPoint);
        std::vector<std::vector<cv::Point2d>>lines;
        MyRansac(VecPoint,lines);
        bool isLocationValued = getRadarPosition(lines,position,mode);
        if(!isLocationValued){cv::imshow("radarImage",image); return false;}
        slidewindow.push_back(position);
        averageFilter(slidewindow,position);
        if(slidewindow.size()>WINDOWSIZE)
            slidewindow.pop_front();
        // view
        std::stringstream text;
        text<<"radar(x,corner) = ( "<<position.front()<<","<<position.back()<<")";
        std::stringstream value;
        value <<"isvalued :"<<"yes";
        cv::putText(image,text.str(),cv::Point(20,50),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(194,20,93),1);
        cv::putText(image,value.str(),cv::Point(20,80),cv::FONT_HERSHEY_PLAIN,2,cv::Scalar(87,45,200),1);
        if(Param::radarLocation.debug)Param::mimshow("radarImage",image);
        return true;
    }
    bool RadarController::getRadarPosition(std::vector<std::vector<cv::Point2d> >&lines,std::vector<float > &position,char mode)
    {
        std::cout<<"getRadarPosition: lines.size = "<<lines.size()<<std::endl;
        if(lines.size()==0) return false;
        std::vector<float>tempL;
        std::vector<float>findL;
        if(mode ==0)
        {
            for(auto l:lines)
            {
                tempL.clear();
                getLocation(l,tempL);
                if(abs(tempL[1]>25)||l.size()<100)continue;
                findL.assign(tempL.begin(),tempL.end());
            }
            getLocation(lines.back(),position);
            //std::cout<<"point "<<lines.back().front()<<std::endl;
            if(abs(position[1])>25){std::cout<<"angle too big "<<position[0]<<","<<position[1]<<std::endl;return false;}
            //std::cout<<"mode = 0 x line.size "<<lines.back().size()<<std::endl;
        }else if(mode ==1)
        {
            for(auto l:lines)
            {
                tempL.clear();
                getLocation(l,tempL);
                if(abs(tempL[1])<65||tempL[0]>2000)continue;
                findL.assign(tempL.begin(),tempL.end());
                //std::cout<<"mode = 1 x line.size "<<l.size()<<std::endl;
                //std::cout<<"mode = 1 angle,dis "<<tempL[1]<<","<<tempL[0]<<std::endl;
            }
            if(findL.empty())return false;
            position.assign(findL.begin(),findL.end());
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
        /*
        if(fitLine(line,kb))
        {
            //std::cout<<"k,b is "<<kb.k<<","<<kb.b<<std::endl;
            x = CalPointLineDistance(cv::Point2d(0,0),kb);
            angle = CalYawAngle(kb);
            position.push_back(x);
            position.push_back(angle);
        }else{
            //std::cout<<"k,b is "<<kb.k<<","<<kb.b<<std::endl;
            x = abs(line.front().x);
            angle = 0;
            position.push_back(x);
            position.push_back(angle);
        }
         */
    }
    void RadarController::averageFilter(std::list<std::vector<float> > &slidewindow,std::vector<float> & position)
    {
        struct{float x;float corner;} sum{0,0};
        for(auto data:slidewindow)
        {
            sum.x+=data[0];
            sum.corner+=data[1];
            //sum.y+=data[2];
        }
        position.clear();
        position.push_back(sum.x/slidewindow.size());
        position.push_back(sum.corner/slidewindow.size());
        //position.push_back(sum.y/slidewindow.size());
    }
}