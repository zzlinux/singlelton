//
// Created by robocon on 18-3-10.
//

#include <iostream>
#include <opencv2/imgproc/imgproc.hpp>
#include "KinectCamera.h"

namespace hitcrt
{
    libfreenect2::Registration* KinectCamera::registration;
    libfreenect2::Frame KinectCamera::undistorted(512,424,4);
    libfreenect2::Frame KinectCamera::registered(512,424,4);
    KinectCamera::KinectCamera():
            listener(types)
    {
        frameFlag = rgbFlag+depthFlag;
        sumRgbDepth = rgbFlag+depthFlag;
        std::string serial = "";
        if(freenect2.enumerateDevices() == 0)
        {
            std::cout << "no device connected!" << std::endl;
            throw(10);
        }
        if (serial == "")
        {
            serial = freenect2.getDefaultDeviceSerialNumber();
        }
        dev = freenect2.openDevice(serial);
        if(dev == 0)
        {
            std::cout << "failure opening device!" << std::endl;
            throw(9);
        }

        dev->setColorFrameListener(&listener);
        dev->setIrAndDepthFrameListener(&listener);

        if (enable_rgb && enable_depth)
        {
            if (!dev->start())
                throw -1;
        }
        else
        {
            if (!dev->startStreams(enable_rgb, enable_depth))
                throw -1;
        }
        std::cout << "device serial: " << dev->getSerialNumber() << std::endl;
        std::cout << "device firmware: " << dev->getFirmwareVersion() << std::endl;

        registration = new libfreenect2::Registration(dev->getIrCameraParams(), dev->getColorCameraParams());
    }
    KinectCamera::~KinectCamera()
    {
        dev->stop();
        dev->close();
        delete registration;
    }
    cv::Mat KinectCamera::getFrameRGB()
    {
        frameSync(rgbFlag);
        cv::Mat color;
        cv::cvtColor(cv::Mat((int)rgb->height, (int)rgb->width, CV_8UC4, rgb->data),color,CV_BGRA2BGR);
        return color;
    }
    cv::Mat KinectCamera::getFrameDepth()
    {
        frameSync(depthFlag);
        cv::Mat dep;
        cv::Mat((int)depth->height,(int)depth->width,CV_32FC1,depth->data).copyTo(dep);
        return dep;
    }
    void KinectCamera::getFrames()
    {
        listener.release(frames);
        if (!listener.waitForNewFrame(frames, 10*1000)) // 10 sconds
        {
            std::cout << "timeout!" << std::endl;
            throw -1;
        }
        rgb = frames[libfreenect2::Frame::Color];
        depth = frames[libfreenect2::Frame::Depth];
    }
    void KinectCamera::frameSync(int id)
    {
        if(frameFlag == id || frameFlag == sumRgbDepth)
        {
            //std::cout<<"frameflag,id,sumRgbDepth: "<<frameFlag<<","<<id<<","<<sumRgbDepth<<std::endl;
            getFrames();
            frameFlag = 0;
        }
        frameFlag +=id;
    }
    void KinectCamera::registrate(cv::Mat &color,cv::Mat &dep,cv::Mat &registrate,cv::Mat &undistort)
    {
        cv::Mat colorbgra(color.rows,color.cols,CV_8UC4);
        cv::Mat A(color.rows,color.cols,CV_8UC1,cv::Scalar(0));
        cv::Mat in[] = {color,A};
        int fromTo[] = {0,0,1,1,2,2,3,3};
        cv::mixChannels(in,2,&colorbgra,1,fromTo,4);
        cv::cvtColor(color,colorbgra,CV_BGR2BGRA);
        libfreenect2::Frame *colorFrame = new libfreenect2::Frame(colorbgra.cols,colorbgra.rows,4,colorbgra.data);
        libfreenect2::Frame *depthFrame = new libfreenect2::Frame(dep.cols,dep.rows,4,dep.data);
        registration->apply(colorFrame, depthFrame, &undistorted, &registered);
        cv::cvtColor(cv::Mat((int)registered.height, (int)registered.width,CV_8UC4, registered.data),registrate,CV_BGRA2BGR);
        cv::Mat(undistorted.height,undistorted.width,CV_32FC1,undistorted.data).copyTo(undistort);
    }
    void KinectCamera::registrate(cv::Mat &color, cv::Mat &dep, cv::Mat &registrate)
    {
        cv::Mat colorbgra(color.rows,color.cols,CV_8UC4);
        cv::Mat A(color.rows,color.cols,CV_8UC1,cv::Scalar(0));
        cv::Mat in[] = {color,A};
        int fromTo[] = {0,0,1,1,2,2,3,3};
        cv::mixChannels(in,2,&colorbgra,1,fromTo,4);
        libfreenect2::Frame *colorFrame = new libfreenect2::Frame(colorbgra.cols,colorbgra.rows,4,colorbgra.data);
        libfreenect2::Frame *depthFrame = new libfreenect2::Frame(dep.cols,dep.rows,4,dep.data);
        registration->apply(colorFrame, depthFrame, &undistorted, &registered);
        cv::cvtColor(cv::Mat((int)registered.height, (int)registered.width,CV_8UC4, registered.data),registrate,CV_BGRA2BGR);
    }
    void KinectCamera::undistortDepth(cv::Mat depth, cv::Mat &depthUndistorted)
    {
        libfreenect2::Frame *depthFrame = new libfreenect2::Frame(depth.cols,depth.rows,4,depth.data);
        registration->undistortDepth(depthFrame,&undistorted);
        cv::Mat(undistorted.height,undistorted.width,CV_32FC1,undistorted.data).copyTo(depthUndistorted);
    }
}
