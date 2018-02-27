#include <iostream>
#include <vector>

#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/visualization/cloud_viewer.h>

#include <opencv2/opencv.hpp>

#include <boost/thread.hpp>
#include <OpenNI.h>

using namespace openni;
using namespace std;

///openni开启摄像头
int main()
{
  double camera_factor = 1000.0;
//  double camera_cx = 256.162;   //kinect 2号
//  double camera_cy = 213.28;
//  double camera_fx = 364.485;
//  double camera_fy = 364.485;

  double camera_cx = 255.5;   //Kinect 1号
  double camera_cy = 204.961;
  double camera_fx = 366.534;
  double camera_fy = 366.534;

  char key = ' ';

  std::ofstream outfile;
  outfile.open("../calibration/ground.txt");

  Status rc = STATUS_OK; // OpenNI函数执行结果

  //OpenNI2图
  VideoFrameRef oniDepthImg, oniColorImg;
  //初始化OpenNI2
  rc = OpenNI::initialize();

  //打开Kinect或Xtion设备
  Device device;
  const char * deviceURL = openni::ANY_DEVICE;  //设备名
 // rc = device.open("freenect2://0?serial=019991643547");
  openni::PlaybackControl* playback;
  rc = device.open("../../oni/color.ONI");
  //rc = device.open(ANY_DEVICE);
  playback = device.getPlaybackControl();
  playback->setSpeed(0.1);
  playback->setRepeatEnabled(true);
  //创建并打开深度数据流
  VideoStream oniDepthStream; //深度数据流

  rc = oniDepthStream.create(device, SENSOR_DEPTH);
  oniDepthStream.setMirroringEnabled(true);
  if( STATUS_OK == rc )
  {
    VideoMode modeDepth;
   // modeDepth.setResolution(512,424/*640,480*/); //分辨率
    modeDepth.setFps(30); //帧率
    modeDepth.setPixelFormat(PIXEL_FORMAT_DEPTH_1_MM); //深度像素格式
    oniDepthStream.setVideoMode(modeDepth);

    oniDepthStream.start(); // 打开深度数据流
    if(STATUS_OK !=  rc)
    {
      std::cerr << "无法打开深度数据流："<<OpenNI::getExtendedError()<<std::endl;
      oniDepthStream.destroy();
    }
  }
  else
  {
    std::cerr << "无法创建深度数据流："<<OpenNI::getExtendedError()<<std::endl;
  }

  VideoStream oniColorStream;  //RGB数据流
  rc = oniColorStream.create(device, SENSOR_COLOR);
  oniColorStream.setMirroringEnabled(true);
  if(STATUS_OK == rc)
  {
    //设置彩色视频模式
    VideoMode modeColor;
    //分辨率
    //modeColor.setResolution(320,240);
    modeColor.setFps(30);//帧率
    modeColor.setPixelFormat(PIXEL_FORMAT_RGB888);
    if(device.isImageRegistrationModeSupported(IMAGE_REGISTRATION_DEPTH_TO_COLOR))
    {
      device.setImageRegistrationMode(IMAGE_REGISTRATION_DEPTH_TO_COLOR); //深度到彩色图配准
    }

    rc = oniColorStream.start();
    if( STATUS_OK != rc )
    {
      std::cerr<< "无法打开彩色数据流："<<OpenNI::getExtendedError()<<std::endl;
      oniColorStream.destroy();
    }
  }
  else
  {
    std::cerr << "无法创建彩色数据流："<<OpenNI::getExtendedError()<<std::endl;
  }

  if (!oniDepthStream.isValid() || !oniColorStream.isValid() )
  {
    std::cerr << "深度数据流不合法"<<std::endl;
    OpenNI::shutdown();
  }

  pcl::visualization::CloudViewer viewer("viewer");

  int i = 0;

  while(key != 'q')
  {
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    const DepthPixel* pDepth = NULL;
    const RGB888Pixel* pColor =NULL;
    ///std::cout << " true " << endl;

    if( STATUS_OK == oniDepthStream.readFrame(&oniDepthImg) && STATUS_OK == oniColorStream.readFrame(&oniColorImg) )
    {
      pDepth =(const DepthPixel*) oniDepthImg.getData();  //获取深度数据
      pColor =(const RGB888Pixel*) oniColorImg.getData(); //获取彩色数据
      for (int m = 0; m < oniDepthImg.getHeight(); m++)
        for(int n = 0; n < oniDepthImg.getWidth(); n++)
        {
          //获取深度图中（m,n）处的值
          double depthv = pDepth[m*oniDepthImg.getWidth() + n];
          if(depthv == 0)
            continue;
          if(fabs(depthv)>7000)continue;
          RGB888Pixel colorv = pColor[m*oniColorImg.getWidth() + n];
          pcl::PointXYZ p;

          double z =  depthv;
          double x = (n - camera_cx) * depthv / camera_fx;
          double y = (m - camera_cy) * depthv / camera_fy;
          p.x = (double)x/camera_factor;
          p.y = (double)y/camera_factor;
          p.z = (double)z/camera_factor;
          //计算这个点的空间坐标
          //CoordinateConverter::convertDepthToWorld(oniDepthStream, n, m, depthv, &(p.x), &(p.y), &(p.z)); //xtion
          //p.x = -(double)p.x/camera_factor;
          //p.y = -(double)p.y/camera_factor;
          //p.z = (double)p.z/camera_factor;
          cloud -> points.push_back(p);
        }
    }

    cloud->height = 1;
    cloud->width = cloud->points.size();
    //std::cout << "point cloud size =" << cloud ->points.size()<<std::endl;
    cloud ->is_dense = false;

    if((cloud ->points.size()) == 0) continue;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_y_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(0.8,2);
    pass.filter(*cloud_y_filtered);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
    sor.setInputCloud (cloud_y_filtered);
    sor.setMeanK (100);
    sor.setStddevMulThresh (1.0);
    sor.filter (*cloud_filtered);


    pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    // Optional
    seg.setOptimizeCoefficients (true);
    // Mandatory
    seg.setModelType (pcl::SACMODEL_PLANE);
    seg.setMethodType (pcl::SAC_RANSAC);
    seg.setDistanceThreshold (0.03);

    seg.setInputCloud (cloud_filtered);
    seg.segment (*inliers, *coefficients);


    //outfile << "cofficients : " << " "
    std::cout << coefficients->values[0] <<" "
              << coefficients->values[1] <<" "
              << coefficients->values[2] <<" "
              << coefficients->values[3] << std::endl;



    // show inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_plane (new pcl::PointCloud<pcl::PointXYZ> ());
    pcl::PointCloud<pcl::PointXYZ>::Ptr plane_flitered (new pcl::PointCloud<pcl::PointXYZ> ());
    extract.filter (*cloud_plane);
    viewer.showCloud(cloud_y_filtered);
    if (cloud_plane->points.empty ())
      std::cerr << "Can't find !" << std::endl;
    else
    {
      std::cerr << " size is" << cloud_plane -> points.size() << std::endl;
      //viewer.showCloud(cloud_filtered);

      for (size_t i = 0; i < cloud_plane->points.size (); i = i + 5)
        outfile  << cloud_plane->points[i].x << " "
                 << cloud_plane->points[i].y << " "
                 << cloud_plane->points[i].z << " " <<std::endl;
    }
    cloud_y_filtered->points.clear();
    cloud -> points.clear();
    cloud_filtered -> points.clear();
    cloud_plane -> points.clear();
    plane_flitered -> points.clear();

    const cv::Mat mImageDepth(
            oniDepthImg.getHeight(), oniDepthImg.getWidth(),
            CV_16UC1, (DepthPixel*)oniDepthImg.getData() );
    // 8c. re-map depth data [0,Max] to [0,255]
    cv::Mat depth8U;
    mImageDepth.convertTo( depth8U, CV_8U  );
    cv::imshow("depth",depth8U);
    key = cv::waitKey(5);
  }
  oniDepthStream.destroy();
  oniColorStream.destroy();
  outfile.close();
  device.close();
  OpenNI::shutdown();
    //fitfloor
  ifstream fin;
  fin.open("../calibration/ground.txt");
  assert(fin.is_open());

  vector<cv::Point3f> v_point3D;


  while(!fin.eof()){
    float x, y, z;
    fin >>x >> y >> z;
    v_point3D.push_back(cv::Point3f(x, y, z));
  }

  struct  timeval st,en;
  gettimeofday(&st, NULL);
  cv::Mat M(v_point3D.size(), 3, CV_32FC1);
  for (size_t i = 0; i < v_point3D.size(); ++i) {
    M.row(i) = cv::Mat(v_point3D[i]).t();
  }
  //cout <<M<<endl;
  cout<<v_point3D.size()<<endl;
  cv::Mat Q(v_point3D.size(), 1, CV_32FC1, cv::Scalar(-1));
//    cout << Q << endl;
//    cout <<M.inv(cv::DECOMP_SVD)<<endl;
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
//    Tcw.rowRange(0,4).col(0) = X;
//
//    cout << Tcw.rowRange(0,4).col(0);
//    Tcw.rowRange(0,4).col(1) = Y;
//    Tcw.rowRange(0,4).col(2) = Z;
//    Tcw.at<float>(3,3) = 1;
//    Tcw.at<float>(1,3) = -1/B;
  cout << Tcw << endl;
  //cv::FileStorage fs("../calibration/RT01.yml",cv::FileStorage::WRITE);
  //fs<<"RT01"<<Tcw;
  //fs.release();
}