#ifndef CANTHREAD_H
#define CANTHREAD_H

#include <thread>
#include "controlcan.h"


//#include "ftsensorInterface.h"
using namespace std;
//using namespace ti5Actuator;
//using namespace kwFTsensor;
//***********包含电机和传感器的CAN接收函数23.3.5************//
struct CanIdData//CAN ID and Data[8]can帧中基本信息
{
    UINT Id;
    BYTE IdData[8];    
};
struct CanIdData_batch//CAN ID and Data[8]can帧中基本信息
{
    UINT *Id;
    BYTE **IdData;    
};

// struct CanIdData_batch1//CAN ID and Data[8]can帧中基本信息取7个
// {
//     UINT *Id;
//     BYTE (*IdData)[7];    
// };
struct CanIdData_double//CAN ID and Data[8]can帧中基本信息取
{
    UINT Id[2]={0};
    BYTE   IdData[2][7]={0};    
};


class CANThread : public thread
{

public:
    CANThread();

    void stop();

    //1.打开设备
    bool openDevice(UINT debicIndex);

    //2.初始化CAN
    bool initCAN(UINT debicIndex,UINT baundRate);

    //3.启动CAN
    bool startCAN(UINT debicIndex);

    //4.发送数据
    bool sendData(UINT debicIndex,UINT channel,UINT ID,BYTE remoteFlag,BYTE externFlag,const unsigned char *data,BYTE len);
    bool sendData_batch(UINT debicIndex,UINT channel,UINT* ID,BYTE remoteFlag,BYTE externFlag,unsigned char **data,BYTE len,INT batch);

    //5.关闭设备
    void closeDevice(UINT debicIndex);

    //0.复位设备，  复位后回到3
    bool reSetCAN(UINT debicIndex);

    // //44.接收数据(自加)
    // bool receieveData(UINT channel,UINT ID,BYTE remoteFlag,BYTE externFlag,const unsigned char *data,BYTE len);
    bool ifreceieveDatanum(UINT debicIndex,UINT channel,int num);//接收到的数据数量是否准确
    bool ifreceieveData(UINT debicIndex,UINT channel);//是否接收到数据
    unsigned char **receieveData(UINT debicIndex,UINT channel,UINT *ID);
    unsigned char *receieveData_single(UINT debicIndex,UINT channel,UINT ID);//接收单个数据5字节
    bool receieveData_s8(UINT debicIndex,UINT channel,UINT ID,UCHAR *&canData);//接收单个数据8字节(可用于多线程)
    bool receieveData_s8_thread(UINT debicIndex,UINT channel,UINT ID,UCHAR (&canData)[8]);//还没完全写好
    unsigned char (*receieveData_s8batch(UINT debicIndex,UINT channel,INT batch))[8];//接收批量数据8字节

    bool receieveData_double(UINT debicIndex,UINT channel,UINT ID,CanIdData_double& candata_double);//接收单个数据
    bool receieveData_doublebatch(UINT debicIndex,UINT channel,UINT *ID,CanIdData_double(& candata_double)[3]);//接收批量数据


    // //44.清空数据(自加)
    bool clearBuffer(UINT debicIndex,UINT channel);
    
    UINT deviceType=4;
    //UINT debicIndex= 0;//要用2个CAN设备
    UINT baundRate=1000;
    BYTE remoteFlag=0;
    BYTE  externFlag=0;
    BYTE len;
    UINT debicCom;

    bool stopped;

private:
    //void run();
   // void sleep(int msec);

};

#endif // CANTHREAD_H
