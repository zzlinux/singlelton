//
// Created by robocon on 18-2-26.
//

#ifndef ROBOCON_TRACECONTROLLER_H
#define ROBOCON_TRACECONTROLLER_H

#include <boost/thread.hpp>
#include <opencv2/core/core.hpp>
#include "TaskFactory.h"
#include "KinectCamera.h"
namespace hitcrt
{
    class TraceController :public TaskProduct
    {
    public:
        TraceController(){};
        void run();
    private:
        std::list<cv::Mat> depthFrameBuff;
        std::list<cv::Mat> colorFrameBuff;
        std::unique_ptr<KinectCamera> cap;
        boost::shared_mutex kinectlock;
        boost::thread m_traceDataThread;
        boost::thread m_traceProcessThread;
        void m_traceReadFrame();
        void m_traceProcess();
    };
}


#endif //ROBOCON_TRACECONTROLLER_H
