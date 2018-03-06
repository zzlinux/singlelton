//
// Created by robocon on 18-1-14.
//

#ifndef ROBOCON_RADARMODEL_H
#define ROBOCON_RADARMODEL_H

#include <list>
#include <fstream>
#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "lib/Urg_driver.h"
#include "lib/Connection_information.h"
#include "lib/math_utilities.h"

using namespace qrk;

namespace hitcrt
{
    class RadarModel {
    public:
        Urg_driver urg;
        RadarModel();
        ~RadarModel();
        cv::Mat image;
        struct LINEKB{ double k;double b;};
        struct LINE{cv::Point2d PointStart;cv::Point2d PointEnd;};
        void depth2xy(const std::vector<long>&data,std::vector<cv::Point2d> &VecPoint,char mode);
        cv::Point2d world2pixel(cv::Point2d old);
        void fillMat(cv::Mat &img, std::vector<cv::Point2d> &points, cv::Scalar color);
        void myimshow(std::vector<cv::Point2d> &points);
        void drawAxes(cv::Mat &img);
        void showResultImg(std::vector<float> &position,std::vector <std::vector<cv::Point2d>> &lines,bool &isLocationValued);
        float CalYawAngle(LINEKB &KB);
        double CalPointLineDistance(const cv::Point2d &point, const LINE &line);
        double CalPointLineDistance(const cv::Point2d point,const LINEKB &kb);
        bool fitLine(std::vector<cv::Point2d> &line,LINEKB & linekb);
        void MyRansac(std::vector<cv::Point2d> & VecPoint,std::vector<std::vector<cv::Point2d> > &lines);
        /*----------------kalman filter------------------------------------------------------------------------*/
        /* 1 Dimension */
        typedef struct {
            float x;  /* state */
            float A;  /* x(n)=A*x(n-1)+u(n),u(n)~N(0,q) */
            float H;  /* z(n)=H*x(n)+w(n),w(n)~N(0,r)   */
            float q;  /* process(predict) noise convariance */
            float r;  /* measure noise convariance */
            float p;  /* estimated error convariance */
            float gain;
        } kalman1_state;
        void kalman1_init(kalman1_state *state, float init_x, float init_p);
        float kalman1_filter(kalman1_state *state, float z_measure);
        /*----------------kalman filter end------------------------------------------------------------------------*/
        const int WINDOWSIZE = 7;
        void averageFilter(std::list<std::vector<float> > &slidewindow,std::vector<float> & position);
        std::ofstream fout;
        void recorder(std::vector<float> & position);
    };
}

#endif //ROBOCON_RADARMODEL_H
