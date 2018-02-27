//
// Created by robocon on 18-1-27.
//

#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

using namespace std;
using namespace cv;

int main(int ac, char** av)
{
    /*
    string filename = "../calibration/test2.yml";
    { //write
        Mat R = Mat_<uchar>::eye(3, 3),
                T = Mat_<double>::zeros(3, 1);

        FileStorage fs("../calibration/test2.yml", FileStorage::WRITE);

        fs << "iterationNr" << 100;
        fs << "strings" << "[";                              // text - string sequence
        fs << "image1.jpg" << "Awesomeness" << "baboon.jpg";
        fs << "]";                                           // close sequence

        fs << "Mapping";                              // text - mapping
        fs << "{" << "One" << 1;
        fs <<        "Two" << 2 << "}";

        fs << "R" << R;                                      // cv::Mat
        fs << "T" << T;


        fs.release();                                       // explicit close
        cout << "Write Done." << endl;
    }

    {//read
        cout << endl << "Reading: " << endl;
        FileStorage fs;
        fs.open("../calibration/test2.yml", FileStorage::READ);

        int itNr;
        //fs["iterationNr"] >> itNr;
        itNr = (int) fs["iterationNr"];
        cout << itNr;
        if (!fs.isOpened())
        {
            cerr << "Failed to open " << filename << endl;
            return 1;
        }

        FileNode n = fs["strings"];                         // Read string sequence - Get node
        if (n.type() != FileNode::SEQ)
        {
            cerr << "strings is not a sequence! FAIL" << endl;
            return 1;
        }

        FileNodeIterator it = n.begin(), it_end = n.end(); // Go through the node
        for (; it != it_end; ++it)
            cout << (string)*it << endl;


        n = fs["Mapping"];                                // Read mappings from a sequence
        cout << "Two  " << (int)(n["Two"]) << "; ";
        cout << "One  " << (int)(n["One"]) << endl << endl;


        Mat R, T;

        fs["R"] >> R;                                      // Read cv::Mat
        fs["T"] >> T;

        cout << endl
             << "R = " << R << endl;
        cout << "T = " << T << endl << endl;

        //Show default behavior for non existing nodes
        cout << "Attempt to read NonExisting (should initialize the data structure with its default).";
    }

    cout << endl
         << "Tip: Open up " << filename << " with a text editor to see the serialized data." << endl;
    */
    /*FileStorage fs("../dataAnalyse/traces.yml",FileStorage::READ);
    FileNode n;
    n = fs["Time20180127-155421"];
    float delta = n["delta"];
    std::cout<<delta<<std::endl;
    n = n["circle"];
    FileNodeIterator it = n.begin();
    for(;it!=n.end();it++ )
    {
        std::cout<<(double)*it<<std::endl;
    }*/
    FileStorage f("../config/param.yaml",FileStorage::READ);
    FileNode thread;
    thread = f["thread"];
    int a = thread["trace"];
    std::cout<<"trace: "<<a<<std::endl;
    //FileNodeIterator it;
    //it = n.begin();
    //n = *it;
    return 0;

}
