#include <iostream>
#include <fstream>
#include <Eigen/Eigen>
#include <opencv2/opencv.hpp>
#include <sys/time.h>

using namespace std;

int main() {
    ifstream fin;
    fin.open("../ground.txt");
    assert(fin.is_open());

    vector<cv::Point3f> v_point3D;


    while(!fin.eof()){
        float x, y, z;
        fin >>x >> y >> z;
        v_point3D.push_back(cv::Point3f(x, y, z));
    }

    struct  timeval st,en;
    gettimeofday(&st, NULL);
    cv::Mat M(v_point3D.size(), 3, CV_32FC1);
    for (size_t i = 0; i < v_point3D.size(); ++i) {
        M.row(i) = cv::Mat(v_point3D[i]).t();
    }
    //cout <<M<<endl;
    cout<<v_point3D.size()<<endl;
    cv::Mat Q(v_point3D.size(), 1, CV_32FC1, cv::Scalar(-1));
//    cout << Q << endl;
//    cout <<M.inv(cv::DECOMP_SVD)<<endl;
    cv::Mat N =M.inv(cv::DECOMP_SVD);
    cout << "N.size: "<<N.size<<endl;

    cv::Mat x = M.inv(cv::DECOMP_SVD) * Q;
    cout <<  x << endl;
    float A = x.at<float>(0);
    float B = x.at<float>(1);
    float C = x.at<float>(2);


    cv::Mat X(3,1, CV_32FC1);
    X.at<float>(0) = B*B+C*C;
    X.at<float>(1) = -A*B;
    X.at<float>(2) = -A*C;
    X = X/cv::norm(X);

    cv::Mat Y(3,1, CV_32FC1);
    Y.at<float>(0) = 0;
    Y.at<float>(1) = (A*A+B*B+C*C)*C;
    Y.at<float>(2) = -(A*A+B*B+C*C)*B;
    Y = Y/cv::norm(Y);

    cv::Mat Z(3,1, CV_32FC1);
    Z.at<float>(0) = A;
    Z.at<float>(1) = B;
    Z.at<float>(2) = C;
    Z = Z/cv::norm(Z);

    cv::Mat Tcw(4,4,CV_32FC1, cv::Scalar(0));
    X.copyTo(Tcw.col(0).rowRange(0,3));
    Y.copyTo(Tcw.col(1).rowRange(0,3));
    Z.copyTo(Tcw.col(2).rowRange(0,3));
    Tcw.at<float>(3,3) = 1;
    Tcw.at<float>(1,3) = -1/B;
    Tcw = Tcw.inv();

    gettimeofday(&en,NULL);
    cout << en.tv_usec - st.tv_usec << endl;
//    Tcw.rowRange(0,4).col(0) = X;
//
//    cout << Tcw.rowRange(0,4).col(0);
//    Tcw.rowRange(0,4).col(1) = Y;
//    Tcw.rowRange(0,4).col(2) = Z;
//    Tcw.at<float>(3,3) = 1;
//    Tcw.at<float>(1,3) = -1/B;
    cout << Tcw << endl;

}