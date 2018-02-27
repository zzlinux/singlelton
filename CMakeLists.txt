cmake_minimum_required(VERSION 2.7)
project(robocon)

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
        src/trace
        src/cameraLocation
        src/radarLocation
        src/protocol
        src/apriltag
        src/calibration
        ${OPENCV_INCLUDE_DIRS}
        ${OPENNI2_INCLUDE_DIRS}
        ${PCL_INCLUDE_DIRS}
        ${EIGEN3_INCLUDE_DIR}
        ${Boost_INCLUDE_DIRS}
)
ADD_SUBDIRECTORY(src/trace)
ADD_SUBDIRECTORY(src/cameraLocation)
ADD_SUBDIRECTORY(src/radarLocation)
ADD_SUBDIRECTORY(src/thread)
ADD_SUBDIRECTORY(src/protocol)
ADD_SUBDIRECTORY(src/apriltag)
ADD_SUBDIRECTORY(src/calibration)

set(SOURCE_FILES src/main.cpp)
add_executable(robocon ${SOURCE_FILES})
target_link_libraries(robocon
        thread
        calibration
        ${BOOST_LIBRARIES}
        )