set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ../../lib)
add_library(thread
        ThreadController.cpp
        ThreadController.h
        Param.cpp
        )
target_link_libraries(thread
        ${BOOST_LIBRARIES}
        trace
        camera
        protocol
        radar
        apriltag
        )
