//
// Created by robocon on 18-1-13.
//

#ifndef ROBOCON_CAMERAMODEL_H
#define ROBOCON_CAMERAMODEL_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <iostream>
#include <assert.h>
namespace hitcrt
{
    class CameraModel {
    public:
        CameraModel(int id);
        ~CameraModel(){};
        int cameraid;
        static int count;
        struct LINEKB{ double k;double b; };
        struct LINEE{ cv::Point2d PointStart;cv::Point2d PointEnd;};
        cv::VideoCapture cap;
        cv::Mat cameraMatrix, distCoeffs;
        cv::Mat map1,map2;
        void thinImg(cv::Mat & src,std::vector<cv::Point2d> & bonePoints);
        void Ransc(std::vector<cv::Point2d> & VecPoint,std::vector<std::vector<cv::Point>> &lines);
        double calYawAngle(LINEKB & lkb);
        double CalPointLineDistance(const cv::Point2d &point, const LINEE &line);
        cv::Point callCrossPoint(LINEKB & lkb1,LINEKB & lkb2);
    };
}


#endif //ROBOCON_CAMERAMODEL_H
