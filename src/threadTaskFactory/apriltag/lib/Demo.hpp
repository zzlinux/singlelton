#include <iostream>
#include <cstring>
#include <vector>
#include <list>
#include <sys/time.h>
#include <Eigen/Core>
#include <Eigen/Dense>
#include "opencv2/opencv.hpp"

#include <libv4l2.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <errno.h>
#include "TagDetector.h"
#include "Tag16h5.h"
#include "Tag25h7.h"
#include "Tag25h9.h"
#include "Tag36h9.h"
#include "Tag36h11.h"
#include "Serial.h"
#include <cmath>
using namespace std;
class demo{
public:
    AprilTags::TagDetector* m_tagDetector;
    AprilTags::TagCodes m_tagCodes;

    bool m_draw; // draw image and April tag detections?
    bool m_arduino; // send tag detections to serial port?
    bool m_timing; // print timing information for each tag extraction call

    int m_width; // image size in pixels
    int m_height;
    double m_tagSize; // April tag side length in meters of square black frame
    double m_fx; // camera focal length in pixels
    double m_fy;
    double m_px; // camera principal point
    double m_py;

    int m_deviceId; // camera id (in case of multiple cameras)

    list<string> m_imgNames;

    cv::VideoCapture m_cap;

    int m_exposure;
    int m_gain;
    int m_brightness;

    Serial m_serial;
    demo();
    ~demo();
    double tic();
    inline double standardRad(double t);
    void wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll);
    void setTagCodes(string s);
    void setup();
    void print_detection(AprilTags::TagDetection& detection);
    void processImage(cv::Mat& image, cv::Mat& image_gray);
    void loadImages();
    bool isVideo();
    void setupVideo();
    void loop();
};