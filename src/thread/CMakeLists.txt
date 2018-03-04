set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../../lib)
add_library(thread
        ThreadController.cpp
        ThreadController.h
        Param.cpp
        )
if(MANUAL_ROBOT)
    target_link_libraries(thread
            protocol
            radar)
else(MANUAL_ROBOT)
    target_link_libraries(thread
            protocol
            trace
            camera
            apriltag
            )
endif(MANUAL_ROBOT)
