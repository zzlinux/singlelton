//
// Created by robocon on 17-12-19.
//

#ifndef KINECT_TRAJECTORY_H
#define KINECT_TRAJECTORY_H


#include <vector>
#include <pcl/point_types.h>
namespace hitcrt {
    class Trajectory {
    public:
        Trajectory() :isEnd(false),lastSize(0),ncount(0),isNull(false) {};
        ~Trajectory() {};
        void clear();
        double checkPoint(pcl::PointXYZ &p);
        bool isEnd;
        size_t lastSize;
        size_t ncount;
        std::vector<pcl::PointXYZ> points;
        bool isNull;
    };
}

#endif //KINECT_TRAJECTORY_H
