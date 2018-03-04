//
// Created by robocon on 18-1-27.
//

#ifndef ROBOCON_RECORDER_H
#define ROBOCON_RECORDER_H

#include <opencv2/core/core.hpp>
#include <pcl/point_types.h>
namespace hitcrt
{
    class Recorder {
    public:
        Recorder();
        ~Recorder(){t.release();};
        void trace(pcl::PointXYZ &ball,pcl::PointXYZ &circle,std::vector<pcl::PointXYZ>&points,float distCen,char isHit);
    private:
        cv::FileStorage t;
    };
}


#endif //ROBOCON_RECORDER_H
