//
// Created by robocon on 18-1-9.
//

#ifndef ROBOCON_THREADCONTROLLER_H
#define ROBOCON_THREADCONTROLLER_H

#include <iostream>
#include <boost/thread.hpp>

#include "serialapp.h"
#include "TaskFactory.h"
namespace hitcrt {
    class ThreadController
    {
    public:
        ThreadController();
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

        std::unique_ptr<TaskProduct> radarLocation;
        std::unique_ptr<TaskProduct> trace;
        std::unique_ptr<TaskProduct> cameraLocation0;
        std::unique_ptr<TaskProduct> cameraLocation1;
        std::unique_ptr<TaskProduct> aprilTag;
    };
}


#endif //ROBOCON_THREADCONTROLLER_H
