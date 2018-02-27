//
// Created by robocon on 18-1-27.
//

#include "Recorder.h"
#include <boost/date_time/posix_time/posix_time.hpp>
#include <iostream>
namespace hitcrt
{
    Recorder::Recorder()
    {
        t = cv::FileStorage("../src/dataAnalyse/traces.yml",cv::FileStorage::WRITE);
    }
    void Recorder::trace(pcl::PointXYZ &ball, pcl::PointXYZ &circle, std::vector<pcl::PointXYZ> &points,
                         float distCen,char isHit)
    {
        for(auto p:points)
            std::cout<<"x,y,z "<<p.x<<","<<p.y<<","<<p.z<<std::endl;
        std::cout<<"(p,dis) :"<<ball<<"   "<<distCen<<std::endl;
        std::cout<<"(circle) "<<circle<<std::endl;
        std::string strTime = boost::posix_time::to_iso_string(boost::posix_time::second_clock::local_time());

        // 这时候strTime里存放时间的格式是YYYYMMDDTHHMMSS，日期和时间用大写字母T隔开了

        int pos = strTime.find('T');
        strTime.replace(pos,1,std::string("-"));
        std::stringstream time;
        time<<"Time"<<strTime.c_str();
        t<<time.str()<<"{";
        t<<"ishit"<<isHit;
        t<<"delta"<<distCen;
        t<<"circle"<<"[:"<<circle.x<<circle.y<<circle.z<<"]";
        t<<"ball  "<<"[:"<<ball.x<<ball.y<<ball.z<<"]";
        t<<"points"<<"[";
        for(auto p:points)
        {
            t<<"[:"<<p.x<<p.y<<p.z<<"]";
        }
        t<<"]";
        t<<"}";
    }
}
