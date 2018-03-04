//
// Created by robocon on 18-1-9.
//

#ifndef ROBOCON_THREADCONTROLLER_H
#define ROBOCON_THREADCONTROLLER_H

#include <iostream>
#include <boost/thread.hpp>

#include "serialapp.h"
#include "TraceController.h"
#include "CameraController.h"
#include "RadarController.h"
#include "ApriltagController.h"

#include "makeParam.h"
namespace hitcrt {
    class ThreadController
    {
    public:
        void init();
        void run();
    private:

        boost::thread m_communicationThread;
        boost::thread m_mutualThread;
        boost::thread m_traceThread;
        boost::thread m_cameraLocation0Thread;
        boost::thread m_cameraLocation1Thread;
        boost::thread m_radarLocationThread;
        boost::thread m_apriltagThread;

        void m_communication();
        void m_mutual();
        void m_trace();
        void m_cameraLocation0();
        void m_cameraLocation1();
        void m_radarLocation();
        void m_apriltag();

#if ROBOT == 0
        std::unique_ptr<RadarController> radarLocation;
#else
        std::unique_ptr<TraceController> trace;
        std::unique_ptr<CameraController> cameraLocation0;
        std::unique_ptr<CameraController> cameraLocation1;
        std::unique_ptr<ApriltagController> aprilTag;
#endif
    };
}


#endif //ROBOCON_THREADCONTROLLER_H
