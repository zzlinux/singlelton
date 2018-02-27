//
// Created by robocon on 18-1-2.
//

#include "BallAssociate.h"
#include "transformer.h"
#include "../thread/Param.h"
#include <sys/time.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
namespace hitcrt
{
    BallAssociate::BallAssociate()
    {
        Trajectory temp;
        for(size_t i = 0;i<MAXTRACENUM;i++)
            traces.push_back(temp);
    }
    void BallAssociate::apply(cv::Mat &color,std::vector<pcl::PointXYZ> &targets,std::vector<Trajectory> &ballTraces)
    {
        struct timeval st,en;
        gettimeofday(&st,NULL);
        for(size_t i=0;i<traces.size();i++)       // record lase trace size
        {
            if(!traces[i].isNull) continue;
            traces[i].lastSize = traces[i].points.size();
        }
        findMovement(color,targets);
        for(size_t i = 0;i<traces.size();i++)
        {
            if(!traces[i].isNull) continue;
            std::cout<<"trace,ncount,size "<<i<<","<<traces[i].ncount<<","<<traces[i].points.size();
            if(traces[i].lastSize == traces[i].points.size()) traces[i].ncount++;  //update stop record
            else traces[i].ncount = 0;
            if (traces[i].ncount > 3 && traces[i].points.size() < 4)           // remove stop trace
            {
                std::cout<<" remove stop id "<<i<<"  "<<traces[i].points.size();
                clearTrace(i);
            }
            std::cout<<std::endl;
            if (traces[i].ncount > 3 && traces[i].points.size() >= 4)          // get correct trace
            {
                std::cout <<i<< " : get max trace: " <<traces[i].points.size()<< std::endl;
                ballTraces.push_back(traces[i]);
                clearTrace(i);
            }
        }
        gettimeofday(&en,NULL);
        //std::cout<<"apply time is "<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
    }
    void BallAssociate::findMovement(cv::Mat & color,std::vector<pcl::PointXYZ> &targets)
    {
         if(targets.size()>0) std::cout<<"targets "<<targets.size()<<std::endl;
         for(auto p:targets)
         {
             bool flag = false;
             for(size_t i = 0;i<traces.size();i++)        //最优关联
             {
                 if(!traces[i].isNull) continue;
                 std::cout<<"mate trace "<<i<<std::endl;
                 float dx = p.x-traces[i].points.back().x;
                 float dy = p.y-traces[i].points.back().y;
                 float distxy = sqrt(dx*dx+dy*dy);
                 std::cout<<"distxy "<<distxy<<" mindis "<<MINDISXY<<" maxdis "<<MAXDISXY<<std::endl;
                 if(distxy>MINDISXY&&distxy<MAXDISXY&&dy>MINDY)
                 {
                     if(traces[i].points.size()<3)          //轨迹头两个点必须有颜色
                     {
                         if(colorJudge(p,color))            //如果颜色对应认为是球
                         {
                             traces[i].points.push_back(p);
                             std::cout<<i<<" : first two size "<<traces[i].points.size()<<std::endl;
                             flag = true;
                             break;
                         }
                     }
                     else{              //现在可以忍受没有颜色
                         if(colorJudge(p,color))        //颜色判断正确,有0.6概率认为正确
                         {
                             double disfitxy = traces[i].checkPoint(p);
                             std::cout<<"distfitxy,max "<<disfitxy<<","<<MAXPREERROR<<std::endl;
                             if(disfitxy<MAXPREERROR) {
                                 traces[i].points.push_back(p);
                                 std::cout<<i<<" : after two size "<<traces[i].points.size()<<std::endl;
                                 flag = true;
                                 break;
                             }
                         }else      //如果没有颜色观察轨迹趋势
                         {
                             if(traces[i].checkPoint(p)<0.15)
                             {
                                 traces[i].points.push_back(p);
                                 flag = true;
                                 break;
                             }
                         }
                     }
                 }
             }
             if(!flag)          //认为第一帧图像肯定有颜色的    1.没有轨迹 2.没有匹配到轨迹     create new trace
             {
                 // colorJudge
                 if(colorJudge(p,color))createTrace(p);
             }
         }
    }
    bool BallAssociate::colorJudge(pcl::PointXYZ &p, cv::Mat color)
    {
        cv::Mat imgHSV,imgThreshold;
        cv::Point point;
        Transformer::invTrans(p,point);
        cv::cvtColor(color,imgHSV,CV_BGR2HSV);
        if(area == 1||area == 2)
            cv::inRange(imgHSV,cv::Scalar(c.h.min,c.s.min,c.v.min),cv::Scalar(c.h.max,c.s.max,c.v.max),imgThreshold);     //color ball
        else if(area == 3)
            cv::inRange(imgHSV,cv::Scalar(g.h.min,g.s.min,g.v.min),cv::Scalar(g.h.max,g.s.max,g.v.max),imgThreshold);       //gold ball
        cv::erode(imgThreshold, imgThreshold, cv::Mat(), cv::Point(-1, -1), 1);		//腐蚀
        cv::dilate(imgThreshold, imgThreshold, cv::Mat(), cv::Point(-1, -1), 5);	//膨胀
        //cv::imshow("binary",imgThreshold);
        if(imgThreshold.at<uchar>(point.y,point.x)>0){std::cout<<"hellocolor"<<std::endl;return true;}
        else {std::cout<<"byezero"<<std::endl;return false;}
    }
    void BallAssociate::clearTrace(int i)
    {
        traces[i].clear();
        tracesize--;
    }
    void BallAssociate::createTrace(pcl::PointXYZ &p)
    {
        std::cout<<"exist trace,newId "<<tracesize;
        for(size_t i = 0;i<traces.size();i++)
        {
            if(traces[i].isNull) continue;
            std::cout<<" "<<i<<std::endl;
            traces[i].isNull = true;
            traces[i].points.push_back(p);
            tracesize++;
            break;
        }
    }
    void BallAssociate::init(char throwarea)
    {
        area = throwarea;
        for(size_t i = 0;i<traces.size();i++)       //clear trace
        {
            clearTrace(i);
        }
        tracesize = 0;
    }
}