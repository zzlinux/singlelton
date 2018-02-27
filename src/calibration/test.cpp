#include <opencv2/core/core.hpp>
#include <iostream>
using namespace cv;
int main()
{
    FileStorage fs("../calibration/test.yml",FileStorage::WRITE);
    fs<<"traces"<<"[";
    fs<<"{";
    fs<<"time"<<2016;
    fs<<"ishit"<<true;
    fs<<"delta"<<0.25;
    fs<<"points"<<"[";
    fs<<"[:"<<1111<<2<<3<<"]";
    fs<<"[:"<<1<<2<<3<<"]";
    fs<<"[:"<<1<<2<<3<<"]";
    fs<<"[:"<<1<<2<<3<<"]";
    fs<<"[:"<<1<<2<<3<<"]";
    fs<<"]";
    fs<<"}";
    fs<<"]";
    fs.release();
    FileStorage fi("../calibration/test.yml",FileStorage::READ);
    FileNode n;
    n = fi["traces"];
    FileNodeIterator it = n.begin();
    n = *it;
    n = n["points"];
    it = n.begin();
    n = *it;
    it = n.begin();
    int a = *it;
    std::cout<<a<<std::endl;
    return 0;
}