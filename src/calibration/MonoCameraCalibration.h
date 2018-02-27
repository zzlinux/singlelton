//
// Created by robocon on 18-2-5.
//

#ifndef ROBOCON_MONOCAMERACALIBRATION_H
#define ROBOCON_MONOCAMERACALIBRATION_H

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d.hpp>
using namespace cv;
using namespace std;
namespace hitcrt
{
    class MonoCameraCalibration {
    public:
        MonoCameraCalibration(){};
        void run();
    private:
        enum { DETECTION = 0, CAPTURING = 1, CALIBRATED = 2 };
        enum Pattern { CHESSBOARD, CIRCLES_GRID, ASYMMETRIC_CIRCLES_GRID };
        double computeReprojectionErrors(
                const vector<vector<Point3f> >& objectPoints,
                const vector<vector<Point2f> >& imagePoints,
                const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                const Mat& cameraMatrix, const Mat& distCoeffs,
                vector<float>& perViewErrors );
        void calcChessboardCorners(Size boardSize, float squareSize, vector<Point3f>& corners, Pattern patternType = CHESSBOARD);
        bool runCalibration( vector<vector<Point2f> > imagePoints,
                                    Size imageSize, Size boardSize, Pattern patternType,
                                    float squareSize, float aspectRatio,
                                    int flags, Mat& cameraMatrix, Mat& distCoeffs,
                                    vector<Mat>& rvecs, vector<Mat>& tvecs,
                                    vector<float>& reprojErrs,
                                    double& totalAvgErr);
        void saveCameraParams( const string& filename,
                                      Size imageSize, Size boardSize,
                                      float squareSize, float aspectRatio, int flags,
                                      const Mat& cameraMatrix, const Mat& distCoeffs,
                                      const vector<Mat>& rvecs, const vector<Mat>& tvecs,
                                      const vector<float>& reprojErrs,
                                      const vector<vector<Point2f> >& imagePoints,
                                      double totalAvgErr );
        bool readStringList( const string& filename, vector<string>& l );
        bool runAndSave(const string& outputFilename,
                               const vector<vector<Point2f> >& imagePoints,
                               Size imageSize, Size boardSize, Pattern patternType, float squareSize,
                               float aspectRatio, int flags, Mat& cameraMatrix,
                               Mat& distCoeffs, bool writeExtrinsics, bool writePoints );

    };
}


#endif //ROBOCON_MONOCAMERACALIBRATION_H
