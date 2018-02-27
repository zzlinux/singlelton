add_library(calibration
        rgbd_camera.cpp
        RgbdCalibration.cpp
        MonoCameraCalibration.cpp)

target_link_libraries(calibration
        ${OpenNI2_LIBRARIES}
        ${OpenCV_LIBS}
)
