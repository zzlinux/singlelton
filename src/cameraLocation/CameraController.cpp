//
// Created by robocon on 18-1-13.
//

#include "CameraController.h"
#include "../thread/Param.h"

namespace hitcrt
{
    void CameraController::run()
    {
        m_cameraDataThread = boost::thread(boost::bind(&CameraController::m_cameraReadFrame,this));
        m_cameraProcessThread = boost::thread(boost::bind(&CameraController::m_cameraProcess,this));
        m_cameraDataThread.join();
    }
    void CameraController::m_cameraReadFrame()
    {
        std::cout<<"cameraDataThread id "<<m_cameraDataThread.get_id()<<std::endl;
        while (Param::m_process)
        {
            struct timeval st,en;
            gettimeofday(&st,NULL);
            {
                boost::unique_lock<boost::shared_mutex> writelock(cameralock);
                cap>>frame;
            }
            gettimeofday(&en,NULL);
            boost::this_thread::sleep(boost::posix_time::milliseconds(1));
            //std::cout<<"camera write time is "<<(en.tv_usec-st.tv_usec)/1000<<" ms"<<std::endl;
        }
    }
    void CameraController::m_cameraProcess()
    {
        std::cout<<"cameraProcessThread id "<<m_cameraProcessThread.get_id()<<std::endl;
        std::vector<float> position;
        bool isLocationValued;
        while (Param::m_process)
        {
            {
                boost::shared_lock<boost::shared_mutex> readlock(cameralock);
                if(frame.empty())continue;
                readFrame = frame.clone();
            }
            apply(position,isLocationValued);
            if(isLocationValued)
            {
                Param::serial->send(SerialApp::SEND_CAMERA,position);
            }
            position.clear();
        }
    }
    void CameraController::apply(std::vector<float> &data,bool & isLocationValued) {
        isLocationValued = true;
        if (readFrame.empty()) { isLocationValued = false;return;}
        std::vector<cv::Mat> channels;
        cv::remap(readFrame, readFrame, map1, map2, cv::INTER_LINEAR);
        split(readFrame, channels);
        temp = channels.at(2);
        std::stringstream txt,linenum;
        if(cv::threshold(temp, temp, 0, 1, CV_THRESH_OTSU)<150) {isLocationValued=false;txt<<"isSend = false bin";linenum<<"zero line";}    //case no white
        else {
            cv::Mat element = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(15, 15));
            cv::morphologyEx(temp, temp, cv::MORPH_OPEN, element);
            std::vector<cv::Point2d> bonePoints;
            thinImg(temp, bonePoints);
            std::vector<std::vector<cv::Point>> lines;
            Ransc(bonePoints, lines);
            int flag;
            if (getLocation(flag, lines, data) == false) {
                isLocationValued = false;
                txt << "isSend = false get";
            } else { txt << "isSend = true"; }
            /************************view*************************/
            if (flag == 2) {
                std::stringstream text;
                text << "(x,y) : " << data[0] << "," << data[1] << "  angle: " << data[2] << std::endl;
                linenum << "double line";
                cv::putText(readFrame, text.str(), cv::Point(10, 20), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(10, 180, 70), 2);
            } else if (flag = 1){
                isLocationValued = false;
                linenum << "single line" << std::endl;
            }
        }
        cv::putText(readFrame, txt.str(), cv::Point(10, 80), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(10, 90, 255), 2);
        cv::putText(readFrame, linenum.str(), cv::Point(10, 60), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(150, 200, 20), 2);
        if(Param::cameraLocation.debug)Param::mimshow("frame",readFrame);
        if(cv::waitKey(1)=='c')
        {
            cv::waitKey(100000000);
        }
        /***********************view end***********************/
        return ;
    }
    /*
     * @param flag 1:get one line ,2: get two line to find center and yawAngle,3:no lines here wrong condition
     * @param lines two dimision vector segmented by ransac
     */
    bool CameraController::getLocation(int & flag,std::vector<std::vector<cv::Point>> & lines,std::vector<float> &data)
    {
        LINEKB lkb;
        int flagVertical = 65535;
        cv::Vec4f line;
        if(lines.size()==2)
        {
            flag = 2;
            double yawAngle[2];
            LINEKB lkbs[2];
            for(int i=0;i<lines.size();i++)
            {
                cv::fitLine(lines[i],line,CV_DIST_HUBER,0,0.01,0.01);
                if(line[0]!=0)
                {
                    lkb.k = line[1]/line[0];
                    lkb.b = line[3]-lkb.k*line[2];
                    yawAngle[i] = calYawAngle(lkb);
                    lkbs[i] = lkb;
                } else
                {
                    std::cout<<"vertical ~~~~~"<<std::endl;
                    flagVertical = i;
                    yawAngle[i] = 0;
                    lkbs[i] = LINEKB{line[2],line[3]};
                }
            }
            if(flagVertical == 65535)
            {
                /*************************** view **************************/
                if(abs(yawAngle[0])>abs(yawAngle[1])) {
                    /*for(auto p:lines[0])
                    {
                        cv::circle(frame,p,1,Scalar(120,39,170),-1);
                    }
                    for(auto p:lines[1])
                    {
                        cv::circle(frame,p,1,Scalar(50,140,170),-1);
                    }*/
                    cv::line(readFrame, cv::Point(200, lkbs[0].k * 200 + lkbs[0].b), cv::Point(400, lkbs[0].k * 400 + lkbs[0].b),
                             cv::Scalar(0, 0,255 ), 3);
                    cv::line(readFrame, cv::Point((50-lkbs[1].b)/lkbs[1].k, 50), cv::Point((400-lkbs[1].b)/lkbs[1].k,400 ),
                             cv::Scalar(0, 255, 0),3);
                } else
                {
                    /*for(auto p:lines[1])
                    {
                        cv::circle(frame,Point(p.x,p.y),1,Scalar(120,39,170),-1);
                    }
                    for(auto p:lines[0])
                    {
                        cv::circle(frame,Point(p.x,p.y),1,Scalar(50,140,170),-1);
                    }*/
                    cv::line(readFrame, cv::Point((50-lkbs[0].b)/lkbs[0].k, 50), cv::Point((400-lkbs[0].b)/lkbs[0].k,400 ),
                             cv::Scalar(0,255,0), 3);
                    cv::line(readFrame, cv::Point(200, lkbs[1].k * 200 + lkbs[1].b), cv::Point(400, lkbs[1].k * 400 + lkbs[1].b),
                             cv::Scalar(0, 0, 255), 3);
                }
                /*************************** view end **************************/
                cv::Point p = callCrossPoint(lkbs[0],lkbs[1]);
                cv::circle(readFrame,p,5,cv::Scalar(0,0,0),-1);
                data.push_back(static_cast<float>(p.x));
                data.push_back(static_cast<float>(p.y));
                double angle = abs(yawAngle[0])>abs(yawAngle[1])?yawAngle[1]:yawAngle[0];
                data.push_back(static_cast<float>(angle));
            }
            else if(flagVertical==0){
                data.push_back(static_cast<float>(lkbs[0].k));
                data.push_back(static_cast<float>(lkbs[1].k*lkbs[0].k+lkbs[1].b));
                data.push_back(0);
                std::cout<<"vertical"<<std::endl;
            }
            else {      // flag ==1
                data.push_back(static_cast<float>(lkbs[1].k));
                data.push_back(static_cast<float>(lkbs[0].k*lkbs[1].k+lkbs[0].b));
                data.push_back(0);
            }
            std::stringstream txt;
            txt<<"across angle = "<<abs(yawAngle[0]-yawAngle[1]);
            cv::putText(readFrame,txt.str(),cv::Point(10,40),cv::FONT_HERSHEY_PLAIN,1,cv::Scalar(90,150,100),2);
            if(abs(yawAngle[0]-yawAngle[1])<70||abs(yawAngle[0]-yawAngle[1])>110)return false;
        }
        else if(lines.size() ==1)
        {
            flag = 1;
            std::cout <<"straight"<<std::endl;

        }else {flag = 3;return false;}
        return true;
    }
}
