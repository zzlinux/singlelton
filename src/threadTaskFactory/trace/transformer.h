// Copyright HITCRT
// Created by Starry on 3/18/17.

#ifndef CLOSURE_TRANSFORMER_H

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <opencv2/opencv.hpp>

namespace hitcrt {
  class Transformer {
    public:

    static void imgToWorld(std::vector<cv::Point3f> data, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);
    static void imgToWorld(cv::Mat depth_img, pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud);
    static void applyTrans(Eigen::Matrix<double,4,Eigen::Dynamic>& before_mat,
                             pcl::PointCloud<pcl::PointXYZ>::Ptr after_cloud);

    static void invTrans(const pcl::PointXYZ& world_point,cv::Point& img_point);


  };

}

#define CLOSURE_TRANSFORMER_H
#endif //CLOSURE_IMG
