//
// Created by robocon on 18-1-31.
//

#include "RgbdCalibration.h"
#include <iostream>
#include <fstream>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/visualization/cloud_viewer.h>
#include <boost/date_time/posix_time/posix_time.hpp>
namespace hitcrt
{
    RgbdCalibration::RgbdCalibration()
    {
        cap = std::unique_ptr<RGBDcamera>(new RGBDcamera(RGBDcamera::Live_mode,RGBDcamera::Kinect,"../../oni/color.ONI"));
    }
    void RgbdCalibration::run()
    {
        char key = ' ';
        std::cout<<"press 'g' to start calibration ";
        std::cout<<"and 'q' to stop calibration … "<<std::endl;
        while (key != 'g')
        {
            rgb = cap->getFrameRGB();
            dep = cap->getFrameDepth();
            dep.convertTo(depth8U,CV_8UC1);
            cv::imshow("color",rgb);
            cv::imshow("depth",depth8U);
            key = cv::waitKey(1);
            if(key == 'q') break;
        }
        if(key != 'g') return;
        getGroundCloud();
        computeRT01();
    }
    void RgbdCalibration::getGroundCloud()
    {
        std::cout<<"start to get ground cloud … press 'q' to stop record"<<std::endl;
        double camera_factor = 1000.0;
        double camera_cx = 255.5;   //Kinect 1号
        double camera_cy = 204.961;
        double camera_fx = 366.534;
        double camera_fy = 366.534;
        char key = ' ';
        pcl::visualization::CloudViewer view("viewer");
        std::ofstream outfile(ground);
        boost::posix_time::ptime time_now,time_now1;
        boost::posix_time::millisec_posix_time_system_config::time_duration_type time_elapse;
        time_now = boost::posix_time::microsec_clock::universal_time();
        while (key != 'q') {
            time_now1 = boost::posix_time::microsec_clock::universal_time();
            time_elapse = time_now1 - time_now;
            int sec = time_elapse.total_seconds();
            if(sec >=60) break;
            rgb = cap->getFrameRGB();
            dep = cap->getFrameDepth();
            dep.convertTo(depth8U,CV_8UC1);
            cv::imshow("color", rgb);
            cv::imshow("depth", depth8U);
            key = cv::waitKey(1);
            pcl::PointXYZ p;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
            for (int j = 0; j < dep.rows; j++) {
                uint16_t *data = dep.ptr<uint16_t>(j);
                for (int i = 0; i < dep.cols; i++) {
                    float depth = data[i];
                    if (fabs(depth) == 0 || fabs(depth) > 7000)continue;
                    p.z = depth/camera_factor;
                    p.x = ((float) (i - camera_cx) * p.z / camera_fx);
                    p.y = ((float) (j - camera_cy) * p.z / camera_fy);
                    cloud->points.push_back(p);
                }
            }
            cloud->height = 1;
            cloud->width = cloud->points.size();
            cloud->is_dense = false;
            if (cloud->size() == 0)continue;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("y");
            pass.setFilterLimits(0.8, 2);
            pass.filter(*cloud_y_filtered);

            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
            pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
            sor.setInputCloud(cloud_y_filtered);
            sor.setMeanK(100);
            sor.setStddevMulThresh(1.0);
            sor.filter(*cloud_filtered);

            pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
            pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
            pcl::SACSegmentation<pcl::PointXYZ> seg;
            seg.setOptimizeCoefficients(true);
            seg.setModelType(pcl::SACMODEL_PLANE);
            seg.setMethodType(pcl::SAC_RANSAC);
            seg.setDistanceThreshold(0.03);
            seg.setInputCloud(cloud_filtered);
            seg.segment(*inliers, *coefficients);
            pcl::ExtractIndices<pcl::PointXYZ> extract;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZ>);
            extract.setInputCloud(cloud_filtered);
            extract.setIndices(inliers);
            extract.setNegative(false);
            extract.filter(*cloud_plane);

            if(cloud_plane->size()==0){std::cout<<"can't find ground"<<std::endl;continue;}
            for(size_t i = 0;i<cloud_plane->points.size();i+=5)
            {
                outfile<<cloud_plane->points[i].x<<" "
                       <<cloud_plane->points[i].y<<" "
                       <<cloud_plane->points[i].z<<std::endl;
            }
            view.showCloud(cloud_plane);
        }
        outfile.close();
    }
    void RgbdCalibration::computeRT01()
    {
        std::cout<<"start to compute RGBD RT01 …"<<std::endl;
        ifstream fin;
        fin.open(ground);
        assert(fin.is_open());
        std::vector<cv::Point3f> v_point3D;
        while(!fin.eof()){
            float x, y, z;
            fin >>x >> y >> z;
            v_point3D.push_back(cv::Point3f(x, y, z));
        }
        fin.close();
        struct  timeval st,en;
        gettimeofday(&st, NULL);
        cv::Mat M(v_point3D.size(), 3, CV_32FC1);
        for (size_t i = 0; i < v_point3D.size(); ++i) {
            M.row(i) = cv::Mat(v_point3D[i]).t();
        }
        cout<<v_point3D.size()<<endl;
        cv::Mat Q(v_point3D.size(), 1, CV_32FC1, cv::Scalar(-1));
        cv::Mat N =M.inv(cv::DECOMP_SVD);
        cout << "N.size: "<<N.size<<endl;
        cv::Mat x = M.inv(cv::DECOMP_SVD) * Q;
        cout <<  x << endl;

        float A = x.at<float>(0);
        float B = x.at<float>(1);
        float C = x.at<float>(2);


        cv::Mat X(3,1, CV_32FC1);
        X.at<float>(0) = B*B+C*C;
        X.at<float>(1) = -A*B;
        X.at<float>(2) = -A*C;
        X = X/cv::norm(X);

        cv::Mat Y(3,1, CV_32FC1);
        Y.at<float>(0) = 0;
        Y.at<float>(1) = (A*A+B*B+C*C)*C;
        Y.at<float>(2) = -(A*A+B*B+C*C)*B;
        Y = Y/cv::norm(Y);

        cv::Mat Z(3,1, CV_32FC1);
        Z.at<float>(0) = A;
        Z.at<float>(1) = B;
        Z.at<float>(2) = C;
        Z = Z/cv::norm(Z);

        cv::Mat Tcw(4,4,CV_32FC1, cv::Scalar(0));
        X.copyTo(Tcw.col(0).rowRange(0,3));
        Y.copyTo(Tcw.col(1).rowRange(0,3));
        Z.copyTo(Tcw.col(2).rowRange(0,3));
        Tcw.at<float>(3,3) = 1;
        Tcw.at<float>(1,3) = -1/B;
        Tcw = Tcw.inv();

        gettimeofday(&en,NULL);
        cout << en.tv_usec - st.tv_usec << endl;
        cout << Tcw << endl;
        cv::FileStorage fs("../config/RT01.yml",cv::FileStorage::WRITE);
        fs<<"RT01"<<Tcw;
        fs.release();
    }
}