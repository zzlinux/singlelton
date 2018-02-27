//
// Created by robocon on 17-12-21.
//

#include "FitTrace.h"
#include <sys/time.h>
namespace hitcrt
{
    void FitTrace::fitting(Trajectory &trace, cv::Mat &R21, cv::Mat &R31)
    {
        struct timeval en,st;
        gettimeofday(&st,NULL);
        int orderA = 2;
        int orderB = 1;
        cv::Mat A(trace.points.size(),orderA+1,CV_32FC1,cv::Scalar(1));
        cv::Mat B(trace.points.size(),orderB+1,CV_32FC1,cv::Scalar(1));
        cv::Mat Y(trace.points.size(),1,CV_32FC1);
        cv::Mat Z(trace.points.size(),1,CV_32FC1);

        // write to matrix
        for(size_t i=0;i<trace.points.size();i++)
        {
            float * a = A.ptr<float>(i);
            float * b = B.ptr<float>(i);
            float * y = Y.ptr<float>(i);
            float * z = Z.ptr<float>(i);
            for(size_t j=1;j<orderA+1;j++)
            {
                a[j] = pow(trace.points[i].y,j);
            }
            b[1] = trace.points[i].x;
            y[0] = trace.points[i].y;
            z[0] = trace.points[i].z;
        }
        // caculate coefficient
        R31 = A.inv(cv::DECOMP_SVD)*Z;
        R21 = B.inv(cv::DECOMP_SVD)*Y;
        gettimeofday(&en,NULL);
        std::cout<<"--------fitting.time "<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
    }
    void FitTrace::fitting(std::vector<pcl::PointXYZ>&points,cv::Mat &R21)
    {
        struct timeval en,st;
        gettimeofday(&st,NULL);
        int orderB = 1;
        cv::Mat B(points.size(),orderB+1,CV_32FC1,cv::Scalar(1));
        cv::Mat Y(points.size(),1,CV_32FC1);
        // write to matrix
        for(size_t i=0;i<points.size();i++)
        {
            float * b = B.ptr<float>(i);
            float * y = Y.ptr<float>(i);
            b[1] = points[i].x;
            y[0] = points[i].y;
        }
        // caculate coefficient
        R21 = B.inv(cv::DECOMP_SVD)*Y;
        gettimeofday(&en,NULL);
        //std::cout<<"--------fitting.time"<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
    }
    void FitTrace::fitting(std::vector<pcl::PointXYZ>&points,cv::Mat &R31,int order)
    {
        struct timeval en,st;
        gettimeofday(&st,NULL);
        int orderA = 2;
        cv::Mat A(points.size(),orderA+1,CV_32FC1,cv::Scalar(1));
        cv::Mat Z(points.size(),1,CV_32FC1);
        // write to matrix
        for(size_t i=0;i<points.size();i++)
        {
            float * a = A.ptr<float>(i);
            float * z = Z.ptr<float>(i);
            for(size_t j=1;j<orderA+1;j++)
            {
                a[j] = pow(points[i].y,j);
            }
            z[0] = points[i].z;
        }
        // caculate coefficient
        R31 = A.inv(cv::DECOMP_SVD)*Z;
        gettimeofday(&en,NULL);
        std::cout<<"--------fitting.time "<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
    }
}