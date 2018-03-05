//
// Created by robocon on 18-3-5.
//

#include "TaskFactory.h"
#include "TraceController.h"
#include "CameraController.h"
#include "RadarController.h"
#include "ApriltagController.h"
#include "makeParam.h"
namespace hitcrt
{
    void TaskProduct::error()
    {
        std::cout<<"arguments error for TaskFactory::CreateTask"<<std::endl;
    }
    TaskProduct * TaskFactory::CreateTask(std::string task,int id)
    {
        TaskProduct * Task;
#if ROBOT == 0
        if(task == "radarLocation")
            Task = new RadarController();
#else
        if(task == "trace")
            Task = new TraceController();
        else if(task == "apriltag")
            Task = new ApriltagController(id);
        else if(task == "cameraLocation")
            Task = new CameraController(id);
        else Task = NULL;
#endif
        return Task;
    }
}