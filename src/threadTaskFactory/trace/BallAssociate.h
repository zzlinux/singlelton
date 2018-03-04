//
// Created by robocon on 18-1-2.
//

#ifndef TRACK_BALLASSOCIATE_H
#define TRACK_BALLASSOCIATE_H

#include "Trajectory.h"
#include "../thread/Param.h"
#include <vector>
#include <pcl/point_types.h>
#include <opencv2/core/core.hpp>
namespace hitcrt {
    class BallAssociate {
    public:
        BallAssociate();
        std::vector<Trajectory> traces;
        void apply(cv::Mat &color,std::vector<pcl::PointXYZ> &targets,std::vector<Trajectory> &ballTraces);
        void init(char throwarea);
    private:
        void findMovement(cv::Mat &color,std::vector<pcl::PointXYZ> &targets);
        bool colorJudge(pcl::PointXYZ &p, cv::Mat color);
        void clearTrace(int i);
        void createTrace(pcl::PointXYZ &p);
        const size_t MAXTRACENUM = 5;
        const float MAXDISXY = 1;
        const float MINDISXY = 0.15;
        const float MINDY = 0.1;
        const float MAXPREERROR = 0.1;
        int tracesize = 0;
        char area = 1;
        Param::colorhsv c = Param::traceinfo.cball;
        Param::colorhsv g = Param::traceinfo.gball;
    };
}


#endif //TRACK_BALLASSOCIATE_H
