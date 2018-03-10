cmake_minimum_required(VERSION 2.7)

project(robocon)

set(MANUAL_ROBOT false)

set(CMAKE_CXX_STANDARD 11)
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
set(CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)

set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CONFIGURATION_TYPES Debug RelWithDebInfo Release)
set(CMAKE_C_FLAGS_RELEASE  "${CMAKE_C_FLAGS_RELEASE} -Wall -O3 -march=native")
set(CMAKE_C_FLAGS_DEBUG "${CMAKE_C_FLAGS_DEBUG} -Wall -pg -march=native")
set(CMAKE_CXX_FLAGS_RELEASE   "${CMAKE_CXX_FLAGS_RELEASE} -Wall -O3 -march=native")
set(CMAKE_CXX_FLAGS_DEBUG    "${CMAKE_CXX_FLAGS_DEBUG} -Wall -pg -march=native")

find_package(OpenCV 3.1.0 REQUIRED)
find_package(Boost  REQUIRED)
find_package(OpenNI2 REQUIRED)
find_package(PCL 1.7 REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
        config
        src/protocol
        src/calibration
        src/thread
        src/threadTaskFactory
        src/threadTaskFactory/trace
        src/threadTaskFactory/cameraLocation
        src/threadTaskFactory/radarLocation
        src/threadTaskFactory/apriltag
        ${OPENCV_INCLUDE_DIRS}
        ${OPENNI2_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
)
ADD_SUBDIRECTORY(src/protocol)
ADD_SUBDIRECTORY(src/thread)
ADD_SUBDIRECTORY(src/threadTaskFactory)
if(MANUAL_ROBOT)
    ADD_SUBDIRECTORY(src/threadTaskFactory/radarLocation)
else(MANUAL_ROBOT)
    ADD_SUBDIRECTORY(src/calibration)
    ADD_SUBDIRECTORY(src/threadTaskFactory/trace)
    ADD_SUBDIRECTORY(src/threadTaskFactory/cameraLocation)
    ADD_SUBDIRECTORY(src/threadTaskFactory/apriltag)
endif(MANUAL_ROBOT)

set(SOURCE_FILES src/main.cpp)
add_executable(robocon ${SOURCE_FILES})
target_link_libraries(robocon
        thread
        ${BOOST_LIBRARIES}
        )
if(NOT MANUAL_ROBOT)
    target_link_libraries(robocon calibration)
endif(NOT MANUAL_ROBOT)
