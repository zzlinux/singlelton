#ifndef crt_SERIALAPP_H_
#define crt_SERIALAPP_H_

#include <boost/thread/mutex.hpp>
#include <string>
#include "serialbase.h"
#include "utility.h"

namespace hitcrt
{

class SerialApp
{
public:
    union Float2uchar
    {
        float fl;
        unsigned char ch[4];
    };
    enum SEND_FLAG
    {
        SEND_HEART_BEAT,
        SEND_ERR_U,
        SEND_RUNE,
        SEND_ERR_UV,
        SEND_RADAR,
        SEND_SUPPLY,
        SEND_CAMERA,
        SEND_TRACE
    };

    enum RECEIVE_FLAG
    {
        RECEIVE_HEART_BEAT,
        RECEIVE_SHUT_DOWN,
        RECEIVE_RADAR,
        RECEIVE_TRACE
    };





    SerialApp(const std::string str, int baudrate);
    ~SerialApp();
    bool decode(unsigned char* buff, size_t& received_length, std::vector<float>& num, SerialApp::RECEIVE_FLAG& receive_flag);
    bool decode_num(unsigned char buff, float& num);
    void send(SEND_FLAG send_flag, std::vector<float> num = std::vector<float>());
    void receive(SerialApp::RECEIVE_FLAG& receive_flag, std::vector<float>& num);

private:
    SerialBase* m_serialBase;
    unsigned char FIRST_ONE;
    unsigned char FIRST_TWO;
    unsigned char LAST_TWO;
    unsigned char LAST_ONE;
    unsigned char MAX_RECEIVE_FLOAT_LENGTH;

/// 线程锁，很重要！！！！！！
/// 避免多个线程同时发送数据
    boost::mutex m_send_mutex;




};


}


#endif // crt_SERIALAPP_H_
