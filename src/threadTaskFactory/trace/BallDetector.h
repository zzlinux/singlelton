//
// Created by robocon on 18-1-1.
//

#ifndef TRACK_BALLDECTOR_H
#define TRACK_BALLDECTOR_H

#include "opencv2/video/background_segm.hpp"
#include <opencv2/core/core.hpp>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
namespace hitcrt {
    class BallDetector {
    public:
        BallDetector();
        void detector(cv::Mat &color,cv::Mat &depth,pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud,std::vector<pcl::PointXYZ>& targets);
        void init(char throwarea);
    private:
        struct searchRange{
            struct {float min;float max;}x,y,z;
        };
        searchRange r[3];
        struct searchRange2d{int l;int r;};
        searchRange2d R[3];
        cv::Ptr<cv::BackgroundSubtractor> bg_model = cv::createBackgroundSubtractorMOG2().dynamicCast<cv::BackgroundSubtractor>();
        int updateNum;
        const int MAXUPDATENUM = 8;
        int area = 1;
    };
}


#endif //TRACK_BALLDECTOR_H
