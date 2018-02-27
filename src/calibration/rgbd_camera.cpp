#include "rgbd_camera.h"

namespace  hitcrt{
  RGBDcamera::RGBDcamera(MODE _mode,Device rgbd, const char* target) {
    m_mode = _mode;
    m_targetFile = target;
    openni::OpenNI::initialize();
    rc = openni::STATUS_OK;
    if(m_mode == RGBDcamera::ONI_mode) {
      rc = camera.open(m_targetFile);
      playback = camera.getPlaybackControl();
      playback->setSpeed(1.0);
      playback->setRepeatEnabled(true);
    } else {
      openni::Array<openni::DeviceInfo> aDeviceList;
      openni::OpenNI::enumerateDevices(&aDeviceList);
      const char* vendor = NULL;
      const char* uri = NULL;
      if(rgbd == RGBDcamera::Kinect) {
        vendor = "Microsoft";
        resolution_width = 512;
        resolution_height = 424;
      }
      if(rgbd == RGBDcamera::Xtion) {
        vendor = "PrimeSense";
        resolution_width = 320;
        resolution_width = 240;
      }
      for (int i = 0; i < aDeviceList.getSize(); ++i) {
        const openni::DeviceInfo& rDevInfo = aDeviceList[i];
        std::cout << "Vendor: " << rDevInfo.getVendor() << " " << vendor << std::endl;
        if (*vendor == *rDevInfo.getVendor()) {
          uri = rDevInfo.getUri();
          break;
        }
      }
      rc = camera.open(uri);
    }
    if(rc != openni::STATUS_OK) {
      std::cerr << "无法打开设备" << std::endl;
      exit(-1);
    }
    rc = m_streamDepth.create(camera, openni::SENSOR_DEPTH);
    m_streamDepth.setMirroringEnabled(true);

    if (rc == openni::STATUS_OK) {
      openni::VideoMode mModeDepth;
      mModeDepth.setResolution(resolution_width,resolution_height);
      mModeDepth.setFps(60);
      mModeDepth.setPixelFormat(openni::PIXEL_FORMAT_DEPTH_1_MM);
      m_streamDepth.setVideoMode(mModeDepth);
      rc = m_streamDepth.start();
      if (rc != openni::STATUS_OK) {
        std::cerr << "无法打开深度数据流：" << openni::OpenNI::getExtendedError() << std::endl;
        m_streamDepth.destroy();
      }
    } else{
      std::cerr << "无法创建深度数据流：" << openni::OpenNI::getExtendedError() << std::endl;
    }

    rc = m_streamColor.create(camera, openni::SENSOR_COLOR);
    m_streamColor.setMirroringEnabled(true);

    if (rc == openni::STATUS_OK) {
      openni::VideoMode mModeColor;
      mModeColor.setFps(60);
      mModeColor.setPixelFormat(openni::PIXEL_FORMAT_RGB888);
      m_streamColor.setVideoMode(mModeColor);
      rc = m_streamColor.start();
      if (rc != openni::STATUS_OK) {
        std::cerr << "无法打开彩色图像数据流：" << openni::OpenNI::getExtendedError() << std::endl;
        m_streamColor.destroy();
      }
    } else{
      std::cerr << "无法创建彩色图像数据流：" << openni::OpenNI::getExtendedError() << std::endl;
    }
    if (!m_streamColor.isValid() || !m_streamDepth.isValid()) {
      std::cerr << "深度数据流不合法" << std::endl;
      openni::OpenNI::shutdown();
      exit(-1);
    }
    //彩色图与深度图对齐
    if (camera.isImageRegistrationModeSupported(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR)) {
      camera.setImageRegistrationMode(openni::IMAGE_REGISTRATION_DEPTH_TO_COLOR);
    }

  }

    void RGBDcamera::showdevice() {
      openni::Array<openni::DeviceInfo> aDeviceList;
      openni::OpenNI::enumerateDevices(&aDeviceList);
      std::cout << "电脑上连接着 " << aDeviceList.getSize() << " 个体感设备." << std::endl;
      for (int i = 0; i < aDeviceList.getSize(); ++i) {
        std::cout << "设备 " << i << std::endl;
        const openni::DeviceInfo& rDevInfo = aDeviceList[i];
        std::cout << "设备名： " << rDevInfo.getName() << std::endl;
        std::cout << "设备Id： " << rDevInfo.getUsbProductId() << std::endl;
        std::cout << "供应商名： " << rDevInfo.getVendor() << std::endl;
        std::cout << "供应商Id: " << rDevInfo.getUsbVendorId() << std::endl;
        std::cout << "设备URI: " << rDevInfo.getUri() << std::endl;
      }

    }

    cv::Mat RGBDcamera::getFrameDepth()  {
      rc = m_streamDepth.readFrame(&frameDepth);
      if (rc == openni::STATUS_OK) {
        m_ImageDepth = cv::Mat(frameDepth.getHeight(), frameDepth.getWidth(), CV_16UC1, (void*)frameDepth.getData());
      }
      return m_ImageDepth.clone();
    }

    cv::Mat RGBDcamera::getFrameRGB(){
      rc = m_streamColor.readFrame(&frameColor);
      if (rc == openni::STATUS_OK) {
        m_ImageRGB = cv::Mat(frameColor.getHeight(), frameColor.getWidth(), CV_8UC3, (void*)frameColor.getData());
        // 首先将RGB格式转换为BGR格式
        cvtColor(m_ImageRGB,m_ImageRGB,CV_BGR2RGB);
      }
      return m_ImageRGB.clone();
    }

    openni::Status RGBDcamera::getStatus(){
      return rc;
    }

    RGBDcamera::~RGBDcamera() {
      m_streamDepth.destroy();
      m_streamColor.destroy();
      camera.close();
      openni::OpenNI::shutdown();
    }

    int RGBDcamera::getFrameNum() {
      if( m_mode == ONI_mode)
        return playback->getNumberOfFrames(m_streamDepth);
      else
        return 1000000000;
    }

}
