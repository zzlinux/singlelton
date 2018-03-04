// Copyright HITCRT.
// Created by Starry on 19/4/17.
// Interface class for RGBD device with openni2.

#ifndef HITCRTVISION_RGBD_H
#define HITCRTVISION_RGBD_H

#include <opencv2/opencv.hpp>
#include <openni2/OpenNI.h>
#include <string>

namespace hitcrt{

    class RGBDcamera {
    public:
      typedef enum mode {
        ONI_mode,
        Live_mode
      }MODE;
      typedef enum device {
        Kinect,
        Xtion
      }Device;
      RGBDcamera(MODE _mode,Device rgbd,const char* target = NULL);
      ~RGBDcamera();
      cv::Mat getFrameDepth();
      cv::Mat getFrameRGB();
      int getFrameNum();
      void showdevice();
      openni::VideoStream m_streamDepth;
      openni::VideoStream m_streamColor;
    private:
      openni::Status getStatus();
      cv::Mat m_ImageDepth;
      cv::Mat m_ImageRGB;
      const char* m_targetFile;
      MODE m_mode;
      int resolution_width;
      int resolution_height;
      openni::Status rc;
      openni::Device camera;
      openni::VideoFrameRef  frameDepth;
      openni::VideoFrameRef  frameColor;
      openni::PlaybackControl* playback;
    };


}

#endif //HITCRTVISION_RGBD_H
