//
// Created by robocon on 18-3-5.
//

#ifndef ROBOCON_THREADTASKFACTORY_H
#define ROBOCON_THREADTASKFACTORY_H

#include "iostream"
namespace hitcrt
{
    class TaskProduct {
    public:
        virtual void run() = 0;
        void error();
    };
    class TaskFactory {
    public:
        static TaskProduct * CreateTask(std::string task,int id = 0);
    };
}


#endif //ROBOCON_THREADTASKFACTORY_H
