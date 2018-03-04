//
// Created by robocon on 18-1-1.
//

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/common.h>
#include <pcl/common/centroid.h>
#include "CircleDetector.h"
#include "transformer.h"
#include "../thread/Param.h"
namespace hitcrt
{
    CircleDetector::CircleDetector():radius3d(0.4),radius2d(0),isValued(false){
        center3d = pcl::PointXYZ(0,0,0);
        center2d = cv::Point(0,0);
        cv::Mat circlerange = Param::traceinfo.circle_range;
        r[0] = {{circlerange.at<float>(0,0),circlerange.at<float>(0,1)},
                {circlerange.at<float>(0,2),circlerange.at<float>(0,3)},
                {circlerange.at<float>(0,4),circlerange.at<float>(0,5)}};
        r[1] = {{circlerange.at<float>(1,0),circlerange.at<float>(1,1)},
                {circlerange.at<float>(1,2),circlerange.at<float>(1,3)},
                {circlerange.at<float>(1,4),circlerange.at<float>(1,5)}};
        r[2] = {{circlerange.at<float>(2,0),circlerange.at<float>(2,1)},
                {circlerange.at<float>(2,2),circlerange.at<float>(2,3)},
                {circlerange.at<float>(2,4),circlerange.at<float>(2,5)}};
        cv::Mat circle2D = Param::traceinfo.circle_range2d;
        R[0] = {circle2D.at<int>(0,0),circle2D.at<int>(0,1)};
        R[1] = {circle2D.at<int>(1,0),circle2D.at<int>(1,1)};
        R[2] = {circle2D.at<int>(2,0),circle2D.at<int>(2,1)};
    };
    bool CircleDetector::detector(cv::Mat &depth,pcl::PointCloud<pcl::PointXYZ>::Ptr outCloud,char Area)
    {
        int area = (int)Area-1;
        cv::Mat circleMask(depth.size(),CV_8UC1,cv::Scalar(0));
        cv::rectangle(circleMask,cv::Point(R[area].l,0),cv::Point(R[area].r,depth.rows-1),cv::Scalar(255),-1);
        cv::Mat circledepth;
        depth.copyTo(circledepth,circleMask);
        std::vector<cv::Point3f> pt3d;
        for(int j = 0;j<circledepth.rows;j++)
        {
            uint16_t* data = circledepth.ptr<uint16_t>(j);
            for(int i = 0;i<circledepth.cols;i++)
            {
                float depth = data[i];
                float pz = fabs(depth);
                if(pz>2500&& pz<8*1000.0) pt3d.push_back(cv::Point3f(i,j,pz));
            }
        }
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
        Transformer::imgToWorld(pt3d,cloud);
        if(cloud->size()==0)return false;
        std::cout<<"z limited.size: "<<cloud->points.size()<<std::endl;
        // pass filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_z_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_x_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(r[area].z.min,r[area].z.max);
        pass.filter(*cloud_z_filtered);
        pass.setInputCloud(cloud_z_filtered);
        pass.setFilterFieldName("y");
        pass.setFilterLimits(r[area].y.min,r[area].y.max);
        pass.filter(*cloud_y_filtered);
        pass.setInputCloud(cloud_y_filtered);
        pass.setFilterFieldName("x");
        pass.setFilterLimits(r[area].x.min,r[area].x.max);
        pass.filter(*cloud_x_filtered);
        std::cout<<"gan pass filter.size: "<<cloud_x_filtered->points.size()<<std::endl;
        if(cloud_x_filtered->points.size()==0)return false;
        // radius filter
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_r_filtered(new pcl::PointCloud<pcl::PointXYZ>);
        pcl::RadiusOutlierRemoval<pcl::PointXYZ> rfil;
        rfil.setInputCloud(cloud_x_filtered);
        rfil.setRadiusSearch(0.06);
        rfil.setMinNeighborsInRadius(6);
        rfil.filter(*cloud_r_filtered);
        if(cloud_r_filtered->points.size()==0) return false;
        pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
        tree->setInputCloud(cloud_r_filtered);
        std::vector<pcl::PointIndices> cluster_indices;
        pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
        ec.setClusterTolerance(0.2);
        ec.setMinClusterSize(140);
        ec.setMaxClusterSize(1000);
        ec.setSearchMethod(tree);
        ec.setInputCloud(cloud_r_filtered);
        ec.extract(cluster_indices);
        std::cout<<"gans cluster.size: "<<cluster_indices.size()<<std::endl;
        if(cluster_indices.size()==0) return false;
        size_t circleNum = 0;
        for(std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin();it!= cluster_indices.end();it++) {
            const float maxDeltaX = 0.3;
            const float maxDeltaY = 0.3;
            const float maxDeltaZ = 3.1;
            const float minDeltaZ = 1;
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster(new pcl::PointCloud<pcl::PointXYZ>);
            for (std::vector<int>::const_iterator pit = it->indices.begin(); pit != it->indices.end(); pit++)
                cloud_cluster->points.push_back(cloud_r_filtered->points[*pit]);
            Eigen::Vector4f centroid;
            pcl::compute3DCentroid(*cloud_cluster, centroid);
            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D(*cloud_cluster, minPt, maxPt);
            if ((maxPt.x - minPt.x) > maxDeltaX || (maxPt.y - minPt.y) > maxDeltaY ||
                (maxPt.z - minPt.z) > maxDeltaZ || (maxPt.z - minPt.z) < minDeltaZ)
                continue;
            *outCloud+=*cloud_cluster;
            if(area ==1||area == 2) center3d  = pcl::PointXYZ(centroid[0],centroid[1],2.4);
            else if(area == 3) center3d = pcl::PointXYZ(centroid[0],centroid[1],3.4);
            cv::Point border,borderOut;
            Transformer::invTrans(center3d,center2d);
            Transformer::invTrans(pcl::PointXYZ(centroid[0],centroid[1],center3d.z+0.4),border);
            Transformer::invTrans(pcl::PointXYZ(centroid[0],centroid[1],center3d.z+0.48),borderOut);
            radius2d = center2d.y-border.y;
            radius2dOut = center2d.y-borderOut.y;
            std::cout << "gan cluster.size: " << cloud_cluster->points.size() << std::endl;
            std::cout<<"max(x,y,z): "<<centroid[0]<<","<<centroid[1]<<","<<centroid[2]<<","<<maxPt.z<<std::endl;
            circleNum++;
        }
        if(circleNum!=1)return false;
        isValued = true;
        return true;
    }
}