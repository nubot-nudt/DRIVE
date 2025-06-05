#ifndef FTSENSORINTERFACE_H
#define FTSENSORINTERFACE_H
    
namespace kwFTsensor
{   
    enum Directives
    {
        STOP_SEND_DATA=0x43, //命令传感器停止发送数据0D 0A
        GET_FTSENSOR_DATA_ALWAYS=0x48, //命令传感器连续发送数据0D 0A
        GET_FTSENSOR_DATA_ONCE=0x49, //命令传感器停止发送数据0D 0A
        SET_FTSENSOR_ID=0xDE, //0xDE AA 00 02 00 02 0D 0A
   };
    enum CANparams//常用CAN设置
    {
        DEVICE_TYPE=4,
        //设备型号，使用的为创芯科技CAN分析仪至尊版
        //DEBIC_INDEX=0,
        //设备索引号，默认为0
        BAUND_RATE=1000,
        //波特率，默认为1000Kbps,注意波特
        REMOTE_FLAG=0,
        //电机使用不涉及远程帧，为数据帧
        EXTERN_FLAG=0,
        //电机使用不涉及扩展帧，为标准帧
        LEN=4,//数据长度均为1
    };
}



#endif // FTSENSORINTERFACE_H