//
// Created by robocon on 18-1-14.
//

#include "RadarModel.h"

namespace hitcrt
{
    RadarModel::RadarModel()
    {
        char * tmp[20];
        Connection_information information(0,tmp);
        // Connects to the sensor
        if (!urg.open(information.device_or_ip_name(),
                      information.baudrate_or_port_number(),
                      information.connection_type())) {
            std::cout << "Urg_driver::open(): "
                 << information.device_or_ip_name() << ": " << urg.what() << std::endl;
        }
        urg.set_scanning_parameter(urg.deg2step(-120), urg.deg2step(120), 0);
        urg.start_measurement(Urg_driver::Distance, Urg_driver::Infinity_times, 0);

    }
    RadarModel::~RadarModel()
    {
        if(urg.is_open())
            urg.close();
    }
    void RadarModel::depth2xy(const std::vector<long> &data, std::vector<cv::Point2d> &VecPoint,char mode)
    {
        long min_distance = urg.min_distance();
        long max_distance = urg.max_distance();
        double xMax = 0;double yMax = 0;
        size_t start,end;
        float scale = data.size()/240;
        if(mode == 0)
        {
            start = (int)scale*130;
            end = (int)scale*210;
        }else if(mode == 1)
        {
            start = (int)scale*75;
            end = data.size()-(int)scale*75;
        }
        double radian;
        for(int i=start;i<end;i++)
        {
            long l = data[i];
            if((l<=min_distance)||(l>=max_distance))
                continue;
            radian = urg.index2rad(i);
            cv::Point2d tmp_point;
            tmp_point.x = l*sin(radian);
            tmp_point.y = l*cos(radian);
            if(tmp_point.x>xMax) xMax = tmp_point.x;
            if(tmp_point.y>yMax) yMax = tmp_point.y;
            //cout <<i<<". "<<"theta = "<<radian<<" r = "<<l<<" x= "<<tmp_point.x<<" y= "<<tmp_point.y<<endl;
            //cout <<"theta = "<<radian<<" distance = "<<l<<endl;
            //
            //cout<<"x = "<<tmp_point.x<<" y = "<<tmp_point.y<<endl;
            VecPoint.push_back(tmp_point);
        }
    }
    cv::Point2d RadarModel::world2pixel(cv::Point2d old)
    {
        int divNum = 17  ;
        cv::Point2d trans;
        trans.x = 600+(int)old.x/divNum;
        trans.y = 550-(int)old.y/divNum;
        return trans;

    }
    void RadarModel::fillMat(cv::Mat &img, std::vector<cv::Point2d> &points, cv::Scalar color)
    {
        for(auto p:points)
        {
            p = world2pixel(p);
            if(p.y>=0&&p.y<800&&p.x>=0&&p.x<1200)
            {
                circle(img,cv::Point(p.x,p.y),3,color,-1);
            }
            else
                std::cout <<"p.x = "<<p.x<<" p.y = "<<p.y<<std::endl;
        }
    }
    void RadarModel::myimshow(std::vector<cv::Point2d> &points)
    {
        cv::Mat image = cv::Mat::zeros(800,1200,CV_8UC3);
        cv::Point2d p;
        for(auto point:points)
        {
            p = world2pixel(point);
            if(p.y>=0&&p.y<800&&p.x>=0&&p.x<1200)
                image.at<cv::Vec3b>(p.y,p.x)[1] = 188;
        }
        cv::imshow("img",image);
        cv::waitKey(1);
    }
    float RadarModel::CalYawAngle(LINEKB &KB)
    {
        //std::cout<<"anglex"<<atan(KB.k)/M_PI*180<<std::endl;
        if(KB.k>0)return 90-atan(KB.k)/M_PI*180;
        else return -90-atan(KB.k)/M_PI*180;
    }
    double RadarModel::CalPointLineDistance(const cv::Point2d &point, const LINE &line)
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
    //overload CalPOintLIneDistance
    double RadarModel::CalPointLineDistance(const cv::Point2d point,const LINEKB &kb)
    {
        return abs(kb.k*point.x-point.y+kb.b)/sqrt(kb.k*kb.k+1);
    }
    bool RadarModel::fitLine(std::vector<cv::Point2d> &line, LINEKB &linekb)
    {
        double A=0,B=0,C=0,D=0;
        for(auto point:line)
        {
            A+=point.x*point.x;
            B+=point.x;
            C+=point.x*point.y;
            D+=point.y;
        }
        double div = line.size()*A-B*B;
        if(div != 0)
        {
            linekb.k = (line.size()*C-B*D)/div;
            linekb.b = (A*D-B*C)/div;
        }
        else
        {
            return false;
        }
        return true;
    }
    #define  RANDOMPOINT VecPoint.at(leftPointsIndex.at(rand()%leftPointsIndex.size()))
    #define RANDCOLOR cv::Scalar(rand()/256,rand()/256,rand()/256)
    void RadarModel::MyRansac(std::vector<cv::Point2d> &VecPoint, std::vector <std::vector<cv::Point2d>> &lines)
    {
        const int MAXLINENUM = 2;
        const int ITERANUM = 100;
        const double disDelta = 30;
        const int MINLINELEN = VecPoint.size()*0.3;
        const int MAXNOISENUM = 20;
        std::vector<int> maxLineIndex;           //ransc找到最优模型的索引
        std::vector<std::vector<int>> LINES;        //记录找到的线段的索引
        std::vector<int> VecIndex(VecPoint.size(),0);     //记录点是否被分离出
        std::vector<int> leftPointsIndex;      //记录被分离后剩余点的索引
        std::vector<int> linePointIndex;        //记录每次迭代暂存符合点
        std::vector<cv::Point2d>findLine;
        const std::vector<cv::Scalar> color ={cv::Scalar(255,80,80),cv::Scalar(80,255,80),cv::Scalar(80,80,255)};
        for(int i=0;i<MAXLINENUM;i++)    //最多抽取5条直线
        {
            for(int n=0;n<VecIndex.size();n++)  //找出点集中分离直线点后的结果
                if(VecIndex[n]==0)
                    leftPointsIndex.push_back(n);
            //cout <<"leftpoints.size = "<<leftPointsIndex.size()<<" VecSize = "<<VecIndex.size()<<endl;
            if(leftPointsIndex.size()<MAXNOISENUM)   //提取直线点后剩余点小于阈值认为剩余为噪声
            {
                std::cout <<"isnoise"<<std::endl;
                break;
            }
            for (int k = 0; k <ITERANUM ; ++k)   //迭代取出当前点集中最长线段
            {
                LINE L1={RANDOMPOINT,RANDOMPOINT};          //当前点集中取两个随机点确定一条直线
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
            }
            maxLineIndex.clear();
            leftPointsIndex.clear();
        }
        //std::cout <<"LINES.size()= "<< LINES.size()<<std::endl;
        sort(LINES.begin(),LINES.end());        //按激光雷达扫描顺序给找到点排序
        image = cv::Mat::zeros(800,1200,CV_8UC3);
        LINEKB kb;
        for(int i=0;i<LINES.size();i++)
        {
            for(auto pIndex:LINES[i])
                findLine.push_back(VecPoint[pIndex]);
            lines.push_back(findLine);
            fillMat(image,findLine,i<3?color[i]:RANDCOLOR);
            // fill the line num
            std::stringstream text;
            text<<i;
            cv::Point2d textPos = world2pixel(findLine.front());  //num
            cv::putText(image,text.str(),cv::Point(textPos.x,textPos.y),cv::FONT_HERSHEY_PLAIN,3,cv::Scalar(255,0,0),2);
            cv::Point2d p1 = findLine.front();
            cv::Point2d p2 = findLine.back();
            if(fitLine(findLine,kb))
            {
                p1.y = kb.k * p1.x + kb.b;
                p2.y = kb.k * p2.x + kb.b;
            }
            p1 = world2pixel(p1);
            p2 = world2pixel(p2);
            cv::line(image, cv::Point(p1.x, p1.y), cv::Point(p2.x, p2.y), cv::Scalar(0, 255, 255));
            circle(image, cv::Point(p1.x, p1.y), 4, cv::Scalar(255, 255, 0), -1);
            circle(image, cv::Point(p2.x, p2.y), 4, cv::Scalar(255, 255, 0), -1);
            findLine.clear();
        }
    }
    /*
     * @brief
     *   Init fields of structure @kalman1_state.
     *   I make some defaults in this init function:
     *     A = 1;
     *     H = 1;
     *   and @q,@r are valued after prior tests.
     *
     *   NOTES: Please change A,H,q,r according to your application.
     *
     * @inputs
     *   state - Klaman filter structure
     *   init_x - initial x state value
     *   init_p - initial estimated error convariance
     * @outputs
     * @retval
     */
    void RadarModel::kalman1_init(kalman1_state *state, float init_x, float init_p)
    {
        state->x = init_x;
        state->p = init_p;
        state->A = 1;
        state->H = 1;
        state->q = 0.008;//10e-6  0.008;  /* predict noise convariance */
        state->r = 0.7;//10e-5;      0.7/* measure error convariance */
    }
    /*
     * @brief
     *   1 Dimension Kalman filter
     * @inputs
     *   state - Klaman filter structure
     *   z_measure - Measure value
     * @outputs
     * @retval
     *   Estimated result
     */
    float RadarModel::kalman1_filter(kalman1_state *state, float z_measure)
    {
        /* Predict */
        state->x = state->A * state->x;
        state->p = state->A * state->A * state->p + state->q;  /* p(n|n-1)=A^2*p(n-1|n-1)+q */
        /* Measurement */
        state->gain = state->p * state->H / (state->p * state->H * state->H + state->r);
        state->x = state->x + state->gain * (z_measure - state->H * state->x);
        state->p = (1 - state->gain * state->H) * state->p;
        return state->x;
    }
}