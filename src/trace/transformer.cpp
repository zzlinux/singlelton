#include <math.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "transformer.h"
#include "../thread/Param.h"


namespace hitcrt {

  void Transformer::imgToWorld(cv::Mat depth_img,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud) {
    const int dsize = depth_img.cols*depth_img.rows;
    Eigen::Matrix<double,4,Eigen::Dynamic> data_matrix,out_matrix;
    data_matrix.resize(4, dsize);
    for (int i=0; i<depth_img.rows; ++i) {
      uint16_t* data = depth_img.ptr<uint16_t>(i);
      for (int j=0; j<depth_img.cols; ++j) {
        float depth = data[j];
        float z = depth/Param::CAMERA_FACTOR;
        data_matrix(0,i*depth_img.rows+j) = j*z;
        data_matrix(1,i*depth_img.rows+j) = i*z;
        data_matrix(2,i*depth_img.rows+j) = z;
        data_matrix(3,i*depth_img.rows+j) = 1;
      }
    }
    applyTrans(data_matrix,out_cloud);
  }

  void Transformer::imgToWorld(std::vector<cv::Point3f> data,
                               pcl::PointCloud<pcl::PointXYZ>::Ptr out_cloud) {
    Eigen::Matrix<double,4,Eigen::Dynamic> data_matrix;
    data_matrix.resize(4,data.size());
    for (int i=0; i< static_cast<int>(data.size());++i) {
      float z = data[i].z/Param::CAMERA_FACTOR;
      data_matrix(0,i) = data[i].x*z;
      data_matrix(1,i) = data[i].y*z;
      data_matrix(2,i) = z;
      data_matrix(3,i) = 1;
    }
    applyTrans(data_matrix,out_cloud);
  }

  void Transformer::applyTrans(Eigen::Matrix<double,4,Eigen::Dynamic>& before_mat,
															  pcl::PointCloud<pcl::PointXYZ>::Ptr after_cloud) {

    Eigen::Matrix3d intrinsic_;
    Eigen::Matrix4d rt01_;
    Eigen::Matrix4d trans_matrix_;
    intrinsic_ << Param::FX,  0, Param::CX,
                  0, Param::FY, Param::CY,
                  0,  0,  1;
    rt01_ << Param::RT01.at<float>(0,0), Param::RT01.at<float>(0,1), Param::RT01.at<float>(0,2), Param::RT01.at<float>(0,3),
            Param::RT01.at<float>(1,0), Param::RT01.at<float>(1,1), Param::RT01.at<float>(1,2), Param::RT01.at<float>(1,3),
            Param::RT01.at<float>(2,0), Param::RT01.at<float>(2,1), Param::RT01.at<float>(2,2), Param::RT01.at<float>(2,3),
            Param::RT01.at<float>(3,0), Param::RT01.at<float>(3,1), Param::RT01.at<float>(3,2), Param::RT01.at<float>(3,3);
    Eigen::Matrix4d intrinsic_inv01 = Eigen::Matrix4d::Identity();
    intrinsic_inv01.topLeftCorner(3,3) = intrinsic_.inverse();
    trans_matrix_ = rt01_*intrinsic_inv01;
    Eigen::Matrix<double,4,Eigen::Dynamic> out_matrix;
    out_matrix = trans_matrix_ * before_mat;
    for (int i=0; i<before_mat.cols(); ++i) {
      pcl::PointXYZ p;
      p.x = out_matrix(0,i);
      p.y = out_matrix(1,i);
      p.z = out_matrix(2,i);
      after_cloud->points.push_back(p);
    }
  }

    void Transformer::invTrans(const pcl::PointXYZ& world_point,cv::Point& img_point) {
        Eigen::Matrix3d intrinsic_;
        Eigen::Matrix4d rt01_;
        Eigen::Matrix4d trans_matrix_,trans_inv_;

        intrinsic_ << Param::FX,  0, Param::CX,
                0, Param::FY, Param::CY,
                0,  0,  1;
        rt01_ << Param::RT01.at<float>(0,0), Param::RT01.at<float>(0,1), Param::RT01.at<float>(0,2), Param::RT01.at<float>(0,3),
                Param::RT01.at<float>(1,0), Param::RT01.at<float>(1,1), Param::RT01.at<float>(1,2), Param::RT01.at<float>(1,3),
                Param::RT01.at<float>(2,0), Param::RT01.at<float>(2,1), Param::RT01.at<float>(2,2), Param::RT01.at<float>(2,3),
                Param::RT01.at<float>(3,0), Param::RT01.at<float>(3,1), Param::RT01.at<float>(3,2), Param::RT01.at<float>(3,3);
        Eigen::Matrix4d intrinsic_inv01 = Eigen::Matrix4d::Identity();
        intrinsic_inv01.topLeftCorner(3,3) = intrinsic_.inverse();
        trans_matrix_ = rt01_*intrinsic_inv01;
        trans_inv_ = trans_matrix_.inverse();
        Eigen::Vector4d world_data,img_data;
        world_data << world_point.x, world_point.y, world_point.z, 1;
        img_data = trans_inv_*world_data;
        img_point.x = img_data(0)/img_data(2);
        img_point.y = img_data(1)/img_data(2);
    }


}
