//
// Created by robocon on 17-12-19.
//

#include "Trajectory.h"
#include "FitTrace.h"
namespace hitcrt
{
    void Trajectory::clear()
    {
        points.clear();
        isNull = false;
        ncount = 0;
        lastSize = 0;
    }
    double Trajectory::checkPoint(pcl::PointXYZ &p)
    {
        cv::Mat R21;
        FitTrace::fitting(points,R21);
        double a = R21.at<float>(1);
        double b = R21.at<float>(0);
        //std::cout<<R21<<std::endl;
        //std::cout<<"a,b"<<a<<","<<b<<std::endl;
        return fabs(a*p.x+b-p.y)/sqrt(a*a+1);
    }
}