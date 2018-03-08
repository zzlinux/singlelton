//
// Created by robocon on 18-1-1.
//

#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/centroid.h>
#include <pcl/common/common.h>
#include <iostream>
#include "BallDetector.h"
#include "transformer.h"
#include "../thread/Param.h"
namespace hitcrt
{
    BallDetector::BallDetector():updateNum(0){
        cv::Mat ballrange = Param::traceinfo.ball_range;
        r[0] = {{ballrange.at<float>(0,0),ballrange.at<float>(0,1)},
                {ballrange.at<float>(0,2),ballrange.at<float>(0,3)},
                {ballrange.at<float>(0,4),ballrange.at<float>(0,5)}};
        r[1] = {{ballrange.at<float>(1,0),ballrange.at<float>(1,1)},
                {ballrange.at<float>(1,2),ballrange.at<float>(1,3)},
                {ballrange.at<float>(1,4),ballrange.at<float>(1,5)}};
        r[2] = {{ballrange.at<float>(2,0),ballrange.at<float>(2,1)},
                {ballrange.at<float>(2,2),ballrange.at<float>(2,3)},
                {ballrange.at<float>(2,4),ballrange.at<float>(2,5)}};
        cv::Mat ball2D = Param::traceinfo.ball_range2d;
        R[0] = {ball2D.at<int>(0,0),ball2D.at<int>(0,1)};
        R[1] = {ball2D.at<int>(1,0),ball2D.at<int>(1,1)};
        R[2] = {ball2D.at<int>(2,0),ball2D.at<int>(2,1)};
    };
    void BallDetector::init(char throwarea){area = (int)throwarea-1;updateNum = 0;};
    void BallDetector::detector(cv::Mat &depth,cv::Mat &color, pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud,std::vector<pcl::PointXYZ>& targets)
    {
        /********************remove giant targets**************************/
        cv::Mat depth8U;
        depth.convertTo(depth8U,CV_8UC1);
        cv::erode(depth8U, depth8U, cv::Mat(), cv::Point(-1, -1), 4);		//腐蚀    开运算去除白噪声
        cv::dilate(depth8U, depth8U, cv::Mat(), cv::Point(-1, -1), 4);	//膨胀
        std::vector<std::vector<cv::Point> > contour;
        cv::findContours(depth8U,contour,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
        cv::Mat contMask(depth8U.size(),CV_8UC1,cv::Scalar(255));
        //std::cout<<"contours.size: "<<contour.size()<<std::endl;
        for(size_t i = 0;i<contour.size();i++)
        {
            //std::cout<<std::dec<<contour[i].size()<<std::endl;
            if(contour[i].size()>110)cv::drawContours(contMask,contour,i,cv::Scalar(0),-1);
        }
        /*******************get moving targets*****************************/
        cv::Mat fgmask;
        bg_model->apply(color,fgmask,-1);
        if(updateNum<MAXUPDATENUM){updateNum++;return;}
        cv::erode(fgmask, fgmask, cv::Mat(), cv::Point(-1, -1), 2);		//腐蚀    开运算去除白噪声
        cv::dilate(fgmask, fgmask, cv::Mat(), cv::Point(-1, -1), 4);	//膨胀
        cv::dilate(fgmask, fgmask, cv::Mat(), cv::Point(-1, -1), 4);	//膨胀    闭运算连通白块
        cv::erode(fgmask, fgmask, cv::Mat(), cv::Point(-1, -1), 4);		//腐蚀
        std::vector<std::vector<cv::Point> > contours;
        cv::findContours(fgmask,contours,CV_RETR_EXTERNAL,CV_CHAIN_APPROX_NONE);
        if(contours.size()>0)std::cout<<"ball contour size: "<<contours.size()<<std::endl;
        else return;
        cv::Mat mask(color.size(),CV_8UC1,cv::Scalar(0));
        for(size_t i =0;i<contours.size();i++)
        {
            //std::cout<<"cc size:"<<(int)contours[i].size()<<std::endl;
            if(contours[i].size()<100) {
                cv::drawContours(mask,contours,i,cv::Scalar(255),-1);
            }
        }
        cv::Mat andmask,fgimg,fgcolor;
        andmask = contMask&mask;
        cv::Mat ballMask(depth8U.size(),CV_8UC1,cv::Scalar(0));
        cv::rectangle(ballMask,cv::Point(R[area].l,0),cv::Point(R[area].r,color.rows-1),cv::Scalar(255),-1);
        ballMask = andmask&ballMask;
        depth.copyTo(fgimg,ballMask);
        color.copyTo(fgcolor,ballMask);
        //cv::imshow("ballmask",ballMask);
        //cv::imshow("andmask",andmask);
        //cv::imshow("mask",mask);
        //cv::imshow("fgcolor",fgcolor);
        //cv::imshow("contmask",contMask);
        std::vector<cv::Point3f> pt3d;
        for(int j = 0;j<fgimg.rows;j++)
        {
            uint16_t* data = fgimg.ptr<uint16_t>(j);
            for(int i = 0;i<fgimg.cols;i++)
            {
                float depth = data[i];
                float pz = fabs(depth);
                if(pz>800&& pz<8*1000.0) pt3d.push_back(cv::Point3f(i,j,pz));
            }
        }
        //std::cout<<"z limited.size: "<<pt3d.size()<<std::endl;
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Transformer::imgToWorld(pt3d,cloud);
        if(cloud->size()==0){std::cout<<"ball cloud.size "<<cloud->size()<<std::endl;return;}
        // pass filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        //std::cout<<"z :"<<r[i].z.min<<","<<r[i].z.max<<std::endl;
        pass.setFilterLimits(r[area].z.min,r[area].z.max);
        pass.filter(*cloud_z_filtered);
        pass.setInputCloud(cloud_z_filtered);
        pass.setFilterFieldName("y");
        //std::cout<<"y :"<<r[i].y.min<<","<<r[i].y.max<<std::endl;
        pass.setFilterLimits(r[area].y.min,r[area].y.max);
        pass.filter(*cloud_y_filtered);
        pass.setInputCloud(cloud_y_filtered);
        pass.setFilterFieldName("x");
        //std::cout<<"x :"<<r[i].x.min<<","<<r[i].x.max<<std::endl;
        pass.setFilterLimits(r[area].x.min,r[area].x.max);
        pass.filter(*cloud_x_filtered);
        //std::cout<<"ball pass filter.size: "<<cloud_x_filtered->points.size()<<std::endl;
        if(cloud_x_filtered->points.size()==0)return;
        // radius filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> rfil;
        rfil.setInputCloud(cloud_x_filtered);
        rfil.setRadiusSearch(0.06);
        rfil.setMinNeighborsInRadius(4);
        rfil.filter(*cloud_r_filtered);
        if(cloud_r_filtered->points.size()==0) return;
        //std::cout<<"radius filter.size: "<<cloud_r_filtered->points.size()<<std::endl;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_r_filtered);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.08);
        ec.setMinClusterSize(15);
        ec.setMaxClusterSize(300);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_r_filtered);
        ec.extract(cluster_indices);
        //std::cout<<"cluster.size: "<<cluster_indices.size()<<std::endl;
        outCloud->points.clear();       //clear lase points
        for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it!= cluster_indices.end();it++)
        {
            const float maxDeltaX = 0.2;
            const float maxDeltaY = 0.2;
            const float maxDeltaZ = 0.2;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
            for(std::vector<int>::const_iterator pit = it->indices.begin();pit!=it->indices.end();pit++)
                cloud_cluster->points.push_back(cloud_r_filtered->points[*pit]);
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_cluster,centroid);
            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D (*cloud_cluster, minPt, maxPt);
            if((maxPt.x-minPt.x)>maxDeltaX||(maxPt.y-minPt.y)>maxDeltaY||(maxPt.z-minPt.z)>maxDeltaZ)
            {
                std::cout<<"delta error delta error delta error delta error "<<std::endl;
                continue;
            }
            std::cout<<"cluster.size: "<<cloud_cluster->points.size()<<std::endl;
            targets.push_back(pcl::PointXYZ(centroid[0],centroid[1],centroid[2]));
            *outCloud+=*cloud_cluster;
            //std::cout << "delta(x,y,z) = "
            //          << maxPt.x-minPt.x << ", "
            //          << maxPt.y-minPt.y << ", "
            //          << maxPt.z-minPt.z << ") "<< std::endl;
            std::cout << "The XYZ centroid are: ("
                      << centroid[0] << ", "
                      << centroid[1] << ", "
                      << centroid[2] << ")." << std::endl;
            //std::cout << "The XYZ coordinates of the centroid are: ("
            //          << maxPt.x<< ", "
            //          << maxPt.y<< ", "
            //          << maxPt.z<< ")." << std::endl;
            cv::Point ballCenter;
            for (auto p:targets) {
                Transformer::invTrans(p, ballCenter);
                cv::circle(color, ballCenter, 20, cv::Scalar(255, 255, 255), 1);
            }
        }

    }

}