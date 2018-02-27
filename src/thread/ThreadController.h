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
        boost::thread m_cameraLocationThread;
        boost::thread m_radarLocationThread;
        boost::thread m_apriltagThread;

        void m_communication();
        void m_mutual();
        void m_trace();
        void m_cameraLocation();
        void m_radarLocation();
        void m_apriltag();

        std::unique_ptr<TraceController> trace;
        std::unique_ptr<CameraController> cameraLocation;
        std::unique_ptr<RadarController> radarLocation;
        std::unique_ptr<ApriltagController> aprilTag;
    };
}


#endif //ROBOCON_THREADCONTROLLER_H