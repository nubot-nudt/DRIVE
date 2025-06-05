#include <iostream>
#include "ftsensorPerceptron.h"
#include <time.h>
#include <sstream>
#include <string>
#include <cmath>
#include <unistd.h>
using namespace std;
using namespace kwFTsensor;
// extern geometry_msgs::Wrench ftRmsg[];
// extern geometry_msgs::Wrench ftLmsg[];
extern Sensor sensorR;
extern Sensor sensorL;
// extern std_msgs::Int32 mode;
double last_ForceTorque[6] = {0};

int length(UINT *ID)
{
   return sizeof(ID) / sizeof(ID[0]);
};

// int BYTEtoINT(BYTE *a)//5字节数据的解析参考电机使用文档
// {
//    int la = 0XFFFFFF00 | (*(a));
//    int lb = 0XFFFF00FF | (*(a+1) << 8);
//    int lc = 0XFF00FFFF | (*(a+2) << 16);
//    int ld = 0X00FFFFFF | (*(a+3) << 24);
//    return la&lb&lc&ld;
// }

// BYTE* INTtoBYTE(BYTE mode,int a)//5字节数据的解析参考电机使用文档,加上模式
// {

//    BYTE* r=new BYTE[5];
//    r[0] = mode;
//    r[1] = (BYTE)(a);
//    r[2] = (BYTE)(a>>8);
//    r[3] = (BYTE)(a>>16);
//    r[4] = (BYTE)(a>>24);

//    return r;
// }

double *BYTEtoFT(BYTE *data) // 12字节数据的解析参考传感器使用文档
{

   unsigned int DataTemp = 0;
   static double Force[6] = {0};
   int j = 0;
   for (int i = 0; i < 8; i++)
   {

      DataTemp = data[i + 1];
      if (i % 3 == 0)
      {
         if ((data[i + 1] & 0x80) > 0)
         {
            DataTemp = DataTemp | 0xFFFFFF00; // 留后8位，前面置1，整型补码
         }
         DataTemp = DataTemp << 4;
         DataTemp = DataTemp | (uint)(data[i + 2] >> 4);
         if (j < 3)
         {
            Force[j] = (int)DataTemp * (double)0.002734375; // 11.2/4096
         }
         else
         {
            Force[j] = (int)DataTemp * (double)0.00009765626; // 0.4/4096
         }

         // cout << dec << Force[j] << endl;
         j++;
      }
      else
      {
         if ((data[i + 1] & 0x08) > 0)
         {
            DataTemp = DataTemp | 0xFFFFFFF0; // 留后4位，前面置1，整型补码
         }
         else
         {
            DataTemp = DataTemp & 0x0000000F; // 保留后4位
         }
         DataTemp = DataTemp << 8;
         DataTemp = DataTemp | (uint)(data[i + 2]);
         i = i + 1;
         if (j < 3)
         {
            Force[j] = (int)DataTemp * (double)0.002734375; // 11.2/4096
         }
         else
         {
            Force[j] = (int)DataTemp * (double)0.00009765626; // 0.4/4096
         }
         //   cout << dec << Force[j] << endl;
         j++;
      }
      // DataTemp = DataTemp << i%2*4+4;
      // DataTemp = DataTemp | (uint)(data[i+2] >> 4-i%2*4);
   }
   return Force;
   // printf("Fx= %2f Kg,Fy= %2f Kg,Fz= %2f Kg,Mx= %2f Kg/M,My= %2f Kg/M,Mz= %2f Kg/M\n",Force[0],Force[1],Force[2],Force[3],Force[4],Force[5]);
}

// else if((ReceivedDataLangth>=12)&&(ReceivedDataLangth<120))
// {
//     if(receiveddata.at(0) == 0x0a)
//         receiveddata.pop_front();
//     else
//     {
//         i=0;
//         while ((i<=ReceivedDataLangth-2)&&(receiveddata.at(0)!=0x0d)&&(receiveddata.at(1)!=0x0a))
//         {
//             receiveddata.pop_front();
//             i++;
//         }
//         if(receiveddata.size()>=2)
//         {
//             receiveddata.pop_front();
//             receiveddata.pop_front();
//         }
//     }
// }
// else if(ReceivedDataLangth >= 120)
//     receiveddata.clear();
// else if(ReceivedDataLangth < 12)
//     break;
//   }

bool ftsensorPerceptron::initCan(UINT debicIndex, UINT baundrate)
{
   bool opendevice = ftsensorPerceptron::canthread->openDevice(debicIndex); // 打开CAN
   sleep(1);
   bool initcan = ftsensorPerceptron::canthread->initCAN(debicIndex, baundrate); // 初始化CAN
   sleep(1);
   bool startcan = canthread->startCAN(debicIndex);
   sleep(1);
   bool resetcan = canthread->reSetCAN(debicIndex); // 一般需要重置一下USBCAN,再重启才能正常接收数据
   sleep(1);
   startcan = canthread->startCAN(debicIndex);
   sleep(1);
   if (opendevice && initcan && resetcan && startcan == 1)
   {

      return true;
   }
   else
   {
      cout << "警告,CAN初始化失败！" << endl;
      return false;
   }
}

void ftsensorPerceptron::sendFTdata_always(Sensor sensor) const
{
   unsigned char data[8];
   data[0] = GET_FTSENSOR_DATA_ALWAYS;
   data[1] = 0xAA;
   data[2] = 0x0D;
   data[3] = 0x0A;
   bool send_flag = canthread->sendData(sensor.debicIndex, sensor.channel,
                                        sensor.id,
                                        remoteFlag,
                                        externFlag,
                                        data,
                                        4);
   if (send_flag == 1)
   {
      cout << "always send FTsensor data" << endl;
   }
}

void ftsensorPerceptron::StopFTdata_always(Sensor sensor) const
{
   unsigned char data[8];
   data[0] = STOP_SEND_DATA;
   data[1] = 0xAA;
   data[2] = 0x0D;
   data[3] = 0x0A;
   bool send_flag = canthread->sendData(sensor.debicIndex, sensor.channel,
                                        sensor.id,
                                        remoteFlag,
                                        externFlag,
                                        data,
                                        4);
   if (send_flag == 1)
   {
      cout << "Stop send FTsensor data" << endl;
   }
}

double *ftsensorPerceptron::getFTdata(Sensor sensor)
{
   unsigned char data[8];
   data[0] = 0x49;
   data[1] = 0xAA;
   data[2] = 0x0D;
   data[3] = 0x0A;

   bool send_flag = canthread->sendData(sensor.debicIndex, sensor.channel,
                                        sensor.id,
                                        remoteFlag,
                                        externFlag,
                                        data,
                                        4); // 力传感器的每个设备占两个连续的ID，只用给前一个ID发送读取指令即可获得全部数据

   // unsigned char (*receieve_data)[7];
   CanIdData_double candata_double;
   unsigned char ucharData[12] = {0};
   double *doubleData;
   static double ForceTorque[6];
   if (send_flag)
   {
      if (canthread->receieveData_double(sensor.debicIndex, sensor.channel, sensor.id, candata_double))
      {
         // cout<< "成功接收数据" <<endl;
         for (int j = 0; j < 6; j++)
         {
            ucharData[j] = *(*candata_double.IdData + j + 1);
            ucharData[j + 6] = *(*(candata_double.IdData + 1) + j + 1);
            // cout <<" intucharData" << j <<  " | " << int(ucharData[j])<<  " | " <<" intucharData" << j+6<<  " | " <<  int(ucharData[j+6])<<endl;
         }
         doubleData = BYTEtoFT(ucharData); // 力传感解析
         for (int i = 0; i < 6; i++)
         {
            ForceTorque[i] = doubleData[i]; // 单位kg&kgm
            last_ForceTorque[i] = ForceTorque[i];
         }
         if (sensor.errornum > 0)
         {
            if (sensor.id != 33)
               sensorL.errornum = 0; // 通信畅通了错误数就清0
            else
               sensorR.errornum = 0;
            cout << "ID: " << sensor.id << "成功恢复通讯" << endl;
         }
         return ForceTorque;
      }
      else
         cout << "ID: " << sensor.id << " 通信错误，采用上一帧数据" << endl;
   }
   else
   {
      cout << "ID: " << sensor.id << "发送失败，采用上一帧数据" << endl; // 得记得/9.81不然会出现累乘的情况。
      if (sensor.id != 33)
      {
         for (int i = 0; i < 6; i++)
         {
            ForceTorque[i] = last_ForceTorque[i]; // 单位kg&kgm ，出现数据错误发送上一次的数据
         }
         sensorL.errornum++; // 错误标志位加1
         if (sensorL.errornum > 3)
         {
            cout << "连续3次通讯中断" << endl;
            // mode.data = 11; // 通讯终端进入stop模式
         }
      }
      else
      {
         for (int i = 0; i < 6; i++)
         {
            ForceTorque[i] = last_ForceTorque[i]; // 单位kg&kgm ，出现数据错误发送上一次的数据
         }
         sensorR.errornum++; // 错误标志位加1
         if (sensorR.errornum > 3)
         {
            cout << "连续3次通讯中断" << endl;
            // mode.data = 11; // 通讯终端进入stop模式
         }
      }
      if (canthread->clearBuffer(sensor.debicIndex, sensor.channel))
         cout << "成功清除" << endl;
      return ForceTorque;
   }
}

bool ftsensorPerceptron::startFT(Sensor sensor) const
{

   // cout << 1 << endl;
   unsigned char data[8];
   data[0] = 0x49;
   data[1] = 0xAA;
   data[2] = 0x0D;
   data[3] = 0x0A;
   bool startft;
   for (int i = 0; i < 10; i++)
   {
      bool send_flag = canthread->sendData(sensor.debicIndex, sensor.channel,
                                           sensor.id,
                                           remoteFlag,
                                           externFlag,
                                           data,
                                           4);                // 力传感器的每个设备占两个连续的ID，只用给前一个ID发送读取指令即可获得全部数据
      this_thread::sleep_for(std::chrono::milliseconds(100)); // 延时0.1s，chrono库可做实时控制，最高可精确到纳秒
      startft = canthread->ifreceieveData(sensor.debicIndex, sensor.channel);
      this_thread::sleep_for(std::chrono::milliseconds(100)); // 延时0.1s，chrono库可做实时控制，最高可精确到纳秒

      if (startft)
         break;
   }

   return startft;
}
double (*ftsensorPerceptron::getFTdata_batch(Sensor *sensor, INT batch))[6]
{
   // unsigned char a[4]={GET_FTSENSOR_DATA_ONCE,
   //                                                                                              0xAA,
   //                                                                                              0x0D,
   //                                                                                              0x0A} ;
   // unsigned char *data[3];
   // for (int i = 0; i < 3; i++)
   // {
   //       data[i]=a;
   // }
   unsigned char data[4];
   data[0] = 0x49;
   data[1] = 0xAA;
   data[2] = 0x0D;
   data[3] = 0x0A;
   for (int i = 0; i < batch; i++)
   {
      bool send_flag = canthread->sendData(sensor[i].debicIndex, sensor[i].channel,
                                           sensor[i].id,
                                           remoteFlag,
                                           externFlag,
                                           data,
                                           4);
      this_thread::sleep_for(std::chrono::microseconds(600)); // 传感器要间隔0.2ms发送这样才不会掉帧
   }

   CanIdData_double candata_double[3];
   unsigned char ucharData[12] = {0};
   double *doubleData[3];
   static double ForceTorque[3][6]; // 申请[3][6]的内存，批量数量更改后请做修改
   if (canthread->receieveData_doublebatch(sensor[0].debicIndex, sensor[0].channel, &sensor[0].id, candata_double))
   {
      for (int i = 0; i < 3; i++)
      {
         for (int j = 0; j < 6; j++)
         {
            ucharData[j] = *(*candata_double[i].IdData + j + 1);
            ucharData[j + 6] = *(*(candata_double[i].IdData + 1) + j + 1);
            // cout <<" intucharData" << j <<  " | " << int(ucharData[j])<<  " | " <<" intucharData" << j+6<<  " | " <<  int(ucharData[j+6])<<endl;
         }
         doubleData[i] = BYTEtoFT(ucharData); // 力传感解析

         for (int j = 0; j < 6; j++)
         {
            ForceTorque[i][j] = doubleData[i][j]; // 单位kg&kgm
         }
      }
      return ForceTorque;
   }
   else
   {
      cout << "未成功接收数据" << endl;
   }
}

void ftsensorPerceptron::setFTsensorID(UINT debicIndex, UINT ID, UINT channel, UINT Val) const
{
   unsigned char data[8];
   data[0] = SET_FTSENSOR_ID;
   data[1] = 0xAA;
   data[2] = Val >> 8;
   data[3] = Val;
   data[4] = Val >> 8;
   data[5] = Val;
   data[6] = 0x0D;
   data[7] = 0x0A;
   bool send_flag = canthread->sendData(debicIndex, channel,
                                        ID,
                                        remoteFlag,
                                        externFlag,
                                        data,
                                        4);
   if (send_flag == 1)
   {
      cout << "Set FTsensor id successfully!" << endl;
   }
}
void ftsensorPerceptron::setHomingFTdata(UINT debicIndex, UINT ID, UINT channel, double Val) const
{
}
void ftsensorPerceptron::setBaudrate(UINT debicIndex, UINT ID, UINT channel, UINT Val) const
{
}
ftsensorPerceptron::ftsensorPerceptron(/* args */)
{
   cout << "【NuBot】|力传感器：KW6FTsensor|  |CAN：创芯科技 CAN分析仪顶配版pro|" << endl;
}

ftsensorPerceptron::~ftsensorPerceptron()
{
   canthread->closeDevice(0);
   canthread->closeDevice(1);
   cout << "关闭CAN设备" << endl;
}
