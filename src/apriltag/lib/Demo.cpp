#include <Eigen/Core>
#include <Eigen/Dense>
#include <cmath>
#include <iostream>
#include <sys/time.h>
#include "Demo.hpp"

using namespace std;
const char* windowName = "apriltags_demo";
const double PI = 3.14159265358979323846;
const double TWOPI = 2.0*PI;
demo::demo() : m_tagDetector(NULL),
        m_tagCodes(AprilTags::tagCodes36h11),

        m_draw(true),
        m_arduino(false),
        m_timing(false),

        m_width(640),
        m_height(480),
        m_tagSize(0.166),
        m_fx(600),
        m_fy(600),
        m_px(m_width/2),
        m_py(m_height/2),

        m_exposure(-1),
        m_gain(-1),
        m_brightness(-1),
        m_deviceId(0)
{}
demo::~demo()
{}

double demo::tic() {
    struct timeval t;
    gettimeofday(&t, NULL);
    return ((double)t.tv_sec + ((double)t.tv_usec)/1000000.);
}

inline double demo::standardRad(double t) {
    if (t >= 0.) {
        t = fmod(t+PI, TWOPI) - PI;
    } else {
        t = fmod(t-PI, -TWOPI) + PI;
    }
    return t;
}

void demo::wRo_to_euler(const Eigen::Matrix3d& wRo, double& yaw, double& pitch, double& roll) {
    yaw = standardRad(atan2(wRo(1,0), wRo(0,0)));
    double c = cos(yaw);
    double s = sin(yaw);
    pitch = standardRad(atan2(-wRo(2,0), wRo(0,0)*c + wRo(1,0)*s));
    roll  = standardRad(atan2(wRo(0,2)*s - wRo(1,2)*c, -wRo(0,1)*s + wRo(1,1)*c));
}


void demo::setTagCodes(string s) {
    if (s=="16h5") {
        m_tagCodes = AprilTags::tagCodes16h5;
    } else if (s=="25h7") {
        m_tagCodes = AprilTags::tagCodes25h7;
    } else if (s=="25h9") {
        m_tagCodes = AprilTags::tagCodes25h9;
    } else if (s=="36h9") {
        m_tagCodes = AprilTags::tagCodes36h9;
    } else if (s=="36h11") {
        m_tagCodes = AprilTags::tagCodes36h11;
    } else {
        cout << "Invalid tag family specified" << endl;
        exit(1);
    }
}


void demo::setup() {
    m_tagDetector = new AprilTags::TagDetector(m_tagCodes);

    // prepare window for drawing the camera images
    if (m_draw) {
        cv::namedWindow(windowName, 1);
    }

    // optional: prepare serial port for communication with Arduino
    if (m_arduino) {
        m_serial.open("/dev/ttyACM0");
    }
}


void demo::setupVideo() {


    // find and open a USB camera (built in laptop camera, web cam etc)
    m_cap = cv::VideoCapture(m_deviceId);
    if(!m_cap.isOpened()) {
        cerr << "ERROR: Can't find video device " << m_deviceId << "\n";
        exit(1);
    }
    m_cap.set(CV_CAP_PROP_FRAME_WIDTH, m_width);
    m_cap.set(CV_CAP_PROP_FRAME_HEIGHT, m_height);
    cout << "Camera successfully opened (ignore error messages above...)" << endl;
    cout << "Actual resolution: "
         << m_cap.get(CV_CAP_PROP_FRAME_WIDTH) << "x"
         << m_cap.get(CV_CAP_PROP_FRAME_HEIGHT) << endl;

}

void demo::print_detection(AprilTags::TagDetection& detection) {
    cout << "  Id: " << detection.id
         << " (Hamming: " << detection.hammingDistance << ")";

    // recovering the relative pose of a tag:

    // NOTE: for this to be accurate, it is necessary to use the
    // actual camera parameters here as well as the actual tag size
    // (m_fx, m_fy, m_px, m_py, m_tagSize)

    Eigen::Vector3d translation;
    Eigen::Matrix3d rotation;
    detection.getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                             translation, rotation);

    Eigen::Matrix3d F;
    F <<
      1, 0,  0,
            0,  -1,  0,
            0,  0,  1;
    Eigen::Matrix3d fixed_rot = F*rotation;
    double yaw, pitch, roll;
    wRo_to_euler(fixed_rot, yaw, pitch, roll);

    cout << "  distance=" << translation.norm()
         << "m, x=" << translation(0)
         << ", y=" << translation(1)
         << ", z=" << translation(2)
         << ", yaw=" << yaw
         << ", pitch=" << pitch
         << ", roll=" << roll
         << endl;

    // Also note that for SLAM/multi-view application it is better to
    // use reprojection error of corner points, because the noise in
    // this relative pose is very non-Gaussian; see iSAM source code
    // for suitable factors.
}

void demo::processImage(cv::Mat& image, cv::Mat& image_gray) {
    // alternative way is to grab, then retrieve; allows for
    // multiple grab when processing below frame rate - v4l keeps a
    // number of frames buffered, which can lead to significant lag
    //      m_cap.grab();
    //      m_cap.retrieve(image);

    // detect April tags (requires a gray scale image)
    cv::cvtColor(image, image_gray, CV_BGR2GRAY);
    double t0;
    if (m_timing) {
        t0 = tic();
    }
    vector<AprilTags::TagDetection> detections = m_tagDetector->extractTags(image_gray);
    if (m_timing) {
        double dt = tic()-t0;
        cout << "Extracting tags took " << dt << " seconds." << endl;
    }

    // print out each detection
    cout << detections.size() << " tags detected:" << endl;
    for (int i=0; i<detections.size(); i++) {
        print_detection(detections[i]);
    }

    // show the current image including any detections
    if (m_draw) {
        for (int i=0; i<detections.size(); i++) {
            // also highlight in the image
            detections[i].draw(image);
        }
        imshow(windowName, image); // OpenCV call
    }

    // optionally send tag information to serial port (e.g. to Arduino)
    if (m_arduino) {
        if (detections.size() > 0) {
            // only the first detected tag is sent out for now
            Eigen::Vector3d translation;
            Eigen::Matrix3d rotation;
            detections[0].getRelativeTranslationRotation(m_tagSize, m_fx, m_fy, m_px, m_py,
                                                         translation, rotation);
            m_serial.print(detections[0].id);
            m_serial.print(",");
            m_serial.print(translation(0));
            m_serial.print(",");
            m_serial.print(translation(1));
            m_serial.print(",");
            m_serial.print(translation(2));
            m_serial.print("\n");
        } else {
            // no tag detected: tag ID = -1
            m_serial.print("-1,0.0,0.0,0.0\n");
        }
    }
}

// Load and process a single image
void demo::loadImages() {
    cv::Mat image;
    cv::Mat image_gray;

    for (list<string>::iterator it=m_imgNames.begin(); it!=m_imgNames.end(); it++) {
        image = cv::imread(*it); // load image with opencv
        processImage(image, image_gray);
        while (cv::waitKey(100) == -1) {}
    }
}

// Video or image processing?
bool demo::isVideo() {
    return m_imgNames.empty();
}

// The processing loop where images are retrieved, tags detected,
// and information about detections generated
void demo::loop() {

    cv::Mat image;
    cv::Mat image_gray;

    int frame = 0;
    double last_t = tic();
    while (true) {

        // capture frame
        m_cap >> image;

        processImage(image, image_gray);

        // print out the frame rate at which image frames are being processed
        frame++;
        if (frame % 10 == 0) {
            double t = tic();
            cout << "  " << 10./(t-last_t) << " fps" << endl;
            last_t = t;
        }

        // exit if any key is pressed
        if (cv::waitKey(1) >= 0) break;
    }
}