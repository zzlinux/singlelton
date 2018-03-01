//
// Created by robocon on 18-1-13.
//

#ifndef ROBOCON_CAMERACONTROLLER_H
#define ROBOCON_CAMERACONTROLLER_H

#include "CameraModel.h"
#include <vector>
#include <opencv2/core/core.hpp>
#include <iostream>
#include <boost/thread/thread.hpp>
#include "../thread/Param.h"
namespace hitcrt
{
    class CameraController :private CameraModel
    {
    public:
        CameraController(int id);
        ~CameraController(){};
        void run();
        static int count;
    private:
        std::string windowName;
        cv::Mat frame;
        cv::Mat readFrame;
        cv::Mat temp;
        boost::shared_mutex cameralock;
        boost::thread m_cameraDataThread;
        boost::thread m_cameraProcessThread;
        void m_cameraReadFrame();
        void m_cameraProcess();
        void apply(std::vector<float> &data,bool & isLocationValued);
        bool getLocation(int & flag,std::vector<std::vector<cv::Point>> & lines,std::vector<float> &data);
    };
}


#endif //ROBOCON_CAMERACONTROLLER_H
