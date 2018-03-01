//
// Created by robocon on 18-1-13.
//

#include "CameraModel.h"
#include "../thread/Param.h"

namespace hitcrt
{
    CameraModel::CameraModel(int id):cameraid(id)
    {
        cap = cv::VideoCapture(cameraid);
        assert(cap.isOpened());
        cv::initUndistortRectifyMap(Param::cameraLocationIntrinsic.t(), Param::cameraLocationCoeffs, cv::Mat(), cv::Mat(), cv::Size(640, 480), CV_32F, map1, map2);
    }
    #define  RANDOMPOINT VecPoint.at(leftPointsIndex.at(rand()%leftPointsIndex.size()))
    void CameraModel::Ransc(std::vector<cv::Point2d> & VecPoint,std::vector<std::vector<cv::Point>> &lines)
    {
        const int MAXLINENUM = 2;
        const int ITERANUM = 25;
        const double disDelta = 5;
        const int MINLINELEN = 100;
        const int MAXNOISENUM = 20;
        std::vector<int> maxLineIndex;           //ransc找到最优模型的索引
        std::vector<std::vector<int>> LINES;        //记录找到的线段的索引
        std::vector<int> VecIndex(VecPoint.size(),0);     //记录点是否被分离出
        std::vector<int> leftPointsIndex;      //记录被分离后剩余点的索引
        std::vector<int> linePointIndex;        //记录每次迭代暂存符合点
        std::vector<cv::Point>findLine;
        cv::Mat dst(480,640,CV_8UC3,cv::Scalar(0,0,0));
        for(int i=0;i<MAXLINENUM;i++)    //最多抽取5条直线
        {
            for(int n=0;n<VecIndex.size();n++)  //找出点集中分离直线点后的结果
                if(VecIndex[n]==0)
                    leftPointsIndex.push_back(n);
            //cout <<"leftpoints.size = "<<leftPointsIndex.size()<<endl;
            if(leftPointsIndex.size()<MAXNOISENUM)   //提取直线点后剩余点小于阈值认为剩余为噪声
            {
                std::cout <<"isnoise"<<std::endl;
                break;
            }
            /************************** view **********************************/
            /*
            if(i==1)
                for(auto p:leftPointsIndex)
                {
                    cv::circle(dst,Point(VecPoint[p].x,VecPoint[p].y),2,Scalar(0,100,0),-1);
                }
            else
            for(auto p:leftPointsIndex)
            {
                cv::circle(dst,Point(VecPoint[p].x,VecPoint[p].y),2,Scalar(0,10,200),-1);
            }*/
            /************************ view end ********************************/
            for (int k = 0; k <ITERANUM ; ++k)   //迭代取出当前点集中最长线段
            {
                LINEE L1={RANDOMPOINT,RANDOMPOINT};          //当前点集中取两个随机点确定一条直线
                for(auto j:leftPointsIndex)
                {
                    double dis = CalPointLineDistance(VecPoint[j],L1);
                    if(dis <disDelta)
                    {
                        linePointIndex.push_back(j);
                    }
                }
                if(linePointIndex.size()>maxLineIndex.size())
                {
                    maxLineIndex = linePointIndex;
                }
                linePointIndex.clear();
            }
            if (maxLineIndex.size()< MINLINELEN)                             //从点集中抽离出找到的线段
                break;
            //cout <<"maxline.size() = "<<maxLineIndex.size()<<endl;
            LINES.push_back(maxLineIndex);
            for(auto index:maxLineIndex)
            {
                VecIndex.at(index) = 1;
                //cv::circle(dst,VecPoint[index],1,Scalar(0,100,100),-1);
            }
            maxLineIndex.clear();
            leftPointsIndex.clear();
        }
        for(int i=0;i<LINES.size();i++)
        {
            for(auto pIndex:LINES[i])
                findLine.push_back(cv::Point(VecPoint[pIndex].x,VecPoint[pIndex].y));
            lines.push_back(findLine);
            findLine.clear();
        }
        /************************ view **********************/
        /*for(auto p:lines[0])
        {
            cv::circle(dst,p,1,Scalar(255,0,100),-1);
        }*/
        //cout<<lines[0].size()<<"  "<<lines[1].size()<<endl;
        /*
        if(LINES.size()==2)
        for(auto p:lines[1])
        {
            cv::circle(dst,Point(p.x,p.y),1,Scalar(255,255,255),-1);
        }
        cv::imshow("dst",dst);
        if(waitKey(1)=='c')waitKey(100000);
         */
        /*********************** view end ********************/

    }
    void CameraModel::thinImg(cv::Mat & src,std::vector<cv::Point2d> & bonePoints)
    {
        cv::Mat U(src.size(),CV_8UC1,cv::Scalar(0));
        cv::Mat D(src.size(),CV_8UC1,cv::Scalar(0));
        cv::Mat M(src.size(),CV_8UC1,cv::Scalar(0));
        //double tTime = cv::getTickCount();
        for(int i=0;i<U.rows-1;i++) //U
        {
            uchar * p = U.ptr<uchar>(i);
            uchar * pUp = U.ptr<uchar>(i-1);
            uchar * s = src.ptr<uchar>(i);  //src image
            for(int j=1;j<U.cols-1;j++)
            {
                if(*(s+j)==0)
                {
                    *(p+j) = 0;
                    //cout << static_cast<int>(*(p+j))<<endl;
                }
                else{
                    *(p+j) = std::min(*(p+j-1),std::min(*(pUp+j-1),std::min(*(pUp+j),*(pUp+j+1))))+1;
                    //cout <<static_cast<int>(*(p-src.step+j-1))<<"    "<<static_cast<int>(*(p-src.step+j))<<"    "<<static_cast<int>(*(p-src.step+j+1))<<endl
                    //     <<static_cast<int>(*(p+j-1))<<"    "<<static_cast<int>(*(p+j))<<endl;
                }
            }
        }
        for(int i=D.rows-2;i>0;i--) //D
        {
            uchar * p = D.ptr<uchar>(i);
            uchar * pDown = D.ptr<uchar>(i+1);
            uchar * s = src.ptr<uchar>(i);  //src image
            for(int j=D.cols-2;j>0;j--)
            {
                if(*(s+j)==0) *(p+j) = 0;
                else
                {
                    *(p+j) = std::min(*(p+j+1),std::min(*(pDown+j-1),std::min(*(pDown+j),*(pDown+j+1))))+1;
                    //cout <<"     "<<static_cast<int>(*(p+j))<<"    "<<static_cast<int>(*(p+j+1))<<endl
                    //     <<static_cast<int>(*(p+src.step+j-1))<<"    "<<static_cast<int>(*(p+src.step+j))<<"    "<<static_cast<int>(*(p+src.step+j+1))<<endl;
                }
            }
        }
        for(int i=1;i<M.rows-1;i++) //M
        {
            uchar * m = M.ptr<uchar>(i);
            uchar * u = U.ptr<uchar>(i);
            uchar * d = D.ptr<uchar>(i);
            for(int j = 1;j<M.cols-1;j++)
            {
                *(m+j) = std::min(*(u+j),*(d+j));
            }
        }
        for(int i = 1;i<M.rows-1;i++)
        {
            uchar * m = M.ptr<uchar>(i);
            uchar * mUp = M.ptr<uchar>(i-1);
            uchar * mDown = M.ptr<uchar>(i+1);
            for(int j = 1;j<M.cols-1;j++)
            {
                uchar maxB = std::max(*(m+j-1),std::max(*(mUp+j-1),std::max(*(mUp+j),std::max(*(mUp+j+1),
                                                                          std::max(*(m+j+1),std::max(*(mDown+j+1),std::max(*(mDown+j),*(mDown+j-1))))
                ))));
                /*cout << static_cast<int>(*(m+j-1-dst.step))<<"    "<<static_cast<int>(*(m+j-dst.step))<<"    "<<static_cast<int>(*(m+j+1-dst.step))<<endl
                     << static_cast<int>(*(m+j-1))<<"    "<<static_cast<int>(*(m+j))<<"    "<<static_cast<int>(*(m+j+1))<<endl
                     << static_cast<int>(*(m+j-1+dst.step))<<"    "<<static_cast<int>(*(m+j+dst.step))<<"    "<<static_cast<int>(*(m+j+1+dst.step))<<endl;
                cout <<"maxB = "<< static_cast<int>(maxB)<<endl<<endl;
                 */
                if(*(m+j) == maxB&&maxB>10)
                {
                    bonePoints.push_back(cv::Point2d(static_cast<double>(j), static_cast<double>(i)));
                    //cout<<maxB<<endl;
                }
            }
        }
        //tTime= 1000*((double)cv::getTickCount() - tTime)/cv::getTickFrequency();
        //cout <<"time = "<<tTime<<endl;
        /*Mat dst(M.size(),CV_8UC1,Scalar(0));
        for(auto p:bonePoints)
        {
            dst.at<uchar>(p.y,p.x) = 255;
        }
        cv::imshow("dst",dst);
        waitKey(1);*/
    }
    double CameraModel::calYawAngle(LINEKB & lkb)
    {
        if(lkb.k>0)
            return 90-atan(lkb.k)/M_PI*180;
        else
            return -90-atan(lkb.k)/M_PI*180;
    }
    cv::Point CameraModel::callCrossPoint(LINEKB & lkb1,LINEKB & lkb2)  //x = (b2-b1)/(k1-k2) y = k1*x+b1
    {
        return cv::Point(static_cast<int>((lkb2.b-lkb1.b)/(lkb1.k-lkb2.k)),static_cast<int>(lkb1.k*((lkb2.b-lkb1.b)/(lkb1.k-lkb2.k))+lkb1.b));
    }
    double CameraModel::CalPointLineDistance(const cv::Point2d &point, const LINEE &line)
    {
        //计算直线方程
        if(line.PointStart.x == line.PointEnd.x)//直线斜率无穷大
            return abs(point.x - line.PointStart.x);
        else
        {
            double line_k, line_b;

            line_k = (line.PointStart.y - line.PointEnd.y)/(line.PointStart.x - line.PointEnd.x);
            line_b = line.PointStart.y - line_k*line.PointStart.x;
            return abs(line_k*point.x - point.y + line_b)/sqrt(line_k*line_k + 1);
        }
    }
}