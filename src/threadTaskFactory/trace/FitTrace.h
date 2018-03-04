//
// Created by robocon on 17-12-21.
//

#ifndef KINECT_FITTRACE_H
#define KINECT_FITTRACE_H

#include "Trajectory.h"
#include "opencv2/core/core.hpp"
namespace hitcrt {
    class FitTrace {
    public:
        FitTrace(){};
        ~FitTrace(){};
        static void fitting(Trajectory &trace,cv::Mat &R21,cv::Mat &R31);
        static void fitting(std::vector<pcl::PointXYZ>&points,cv::Mat &R21);
        static void fitting(std::vector<pcl::PointXYZ>&points,cv::Mat &R31,int order);
    };
}


#endif //KINECT_FITTRACE_H
