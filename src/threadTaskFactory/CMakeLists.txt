add_library(taskFactory
        TaskFactory.cpp)

if(MANUAL_ROBOT)
    target_link_libraries(taskFactory
            radar)
else(MANUAL_ROBOT)
    target_link_libraries(taskFactory
            trace
            camera
            apriltag
            )
endif(MANUAL_ROBOT)
