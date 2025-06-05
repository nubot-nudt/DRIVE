#ifndef ACTUATORINTERFACE_H
#define ACTUATORINTERFACE_H
    
namespace ti5Actuator
{   
    enum Directives
    {
GET_ACTUATOR_MODE=0x03,    //获取电机运行模式

GET_CURRENT  =    0x04,    //获取电机电流
GET_VELOCITY =    0x06,    //获取电机速度
GET_POSITION =    0x08,    //获取电机当前位置
GET_CUR_VEL_POS= 0x41,
GET_ERROR_STATE=  0x0A,    //获取电机错误状态
                        /*
                        对应位为0表示无错误，为1表示出现错误
                        bit0代表软件错误，如电机运行时写入FLASH等
                        bit1代表过压
                        bit2代表欠压
                        bit4代表启动错误
                        bit5代表速度反馈错误
                        bit6代表过流
                        bit16代表编码器通讯错误
                        bit17代表电机温度过高
                        bit18代表电路板温度过高
                        */
CLEAR_ERROR   =   0x0B,    //清除电机错误

GET_ACCELERATION= 0x16,    //获取加速度
GET_DECELERATION= 0x17,    //获取减速度

GET_MOTORT_TEMPERATURE= 0x31,    //获取电机温度

GET_PCB_TEMPERATURE=   0x32,    //获取电路板温度


SET_CURRENT=          0x1C,    //设置电机运行模式为电流模式，并设置目标电流
SET_CUR_GET_CUR_VEL_POS=  0x42,    //设置电流并获取电流速度位置
SET_VELOCITY=         0x1D,    //设置电机运行模式为速度模式，并设置目标速度
SET_VEL_GET_CUR_VEL_POS=  0x43,    //设置速度并获取电流速度位置
SET_POSITION =        0x1E,    //设置电机运行模式为位置模式，并设置目标位置
SET_POS_GET_CUR_VEL_POS=  0x44,    //设置位置并获取电流速度位置

SET_MAX_CURRENT=       0x20,      //设置最大正电流
SET_MIN_CURRENT=       0x21,      //设置最小负电流
SET_MAX_ACCELERATION=  0x22,      //设置电机最大正向加速度
SET_MIN_ACCELERATION=  0x23,      //设置电机最小负向加速度
SET_MAX_VELOCITY=      0x24,      //设置最大正向允许速度
SET_MIN_VELOCITY=      0x25,      //设置最小负向允许速度
SET_MAX_POSITION=      0x26,      //设置最大正向位置
SET_MIN_POSITION=      0x27,      //设置最小负向位置

SET_ACTUATOR_ID=       0x2E,      //设置CAN ID 为1到127
SET_HOMING_POSITION=   0x50,      //编码器归零,下发0，设置编码器位置为0，位置偏移为0，编码器报错，需要清除错误
SET_POSITION_OFFSET= 0x53,  //设置位置偏移，设置偏移值和目标位置（目标位置=编码器位置 - 偏移值）
SET_ACTUATOR_EMERGENCY_STOP= 0x52,  //电机急停  （无抱闸固定在当前位置、有抱闸直接抱闸）
SET_BAUDRATE             = 0x3F,  //设置波特率1000、500、250、125、100、50
    };
    enum ErrorsDefine //对应位为0表示无错误，为1表示出现错误
    {
        RR_NONE = 0,
    ///执行器软件错误
        ERR_ACTUATOR_SOFTWARE=0x00,
    ///执行器过压错误
        ERR_ACTUATOR_OVERVOLTAGE=0x01,
    ///执行器欠压错误
        ERR_ACTUATOR_UNDERVOLTAGE=0x02,
    ///执行器启动错误
        ERR_ACTUATOR_LOCKED_ROTOR=0x04,
    ///执行器速度反馈错误
        ERR_ACTUATOR_VELOCITY_FEEDBACK=0x05,
    ///执行器过流错误 
        ERR_ACTUATOR_UNDERCURRENT=0x06,     
    ///编码器通讯错误
        ERR_ENCODE_COMMUNICATION=0x16,
    ///执行器温度过高
        ERR_ACTUATOR_OVERTEMPERATURE=0x17,
    ///电路板温度器过高
        ERR_PCB_OVERTEMPERATURE=0x18,
    };
         enum ActuatorMode
    {
        Mode_Stop=0,
        ///电流模式
        Mode_Cur,
        ///速度模式
        Mode_Vel,
        ///位置模式
        Mode_Pos,
    };
        enum CANparams//电机常用CAN设置
    {
        DEVICE_TYPE=4,
        //设备型号，使用的为创芯科技CAN分析仪至尊版
        DEBIC_INDEX=0,
        //设备索引号，默认为0
        BAUND_RATE=1000,
        //波特率，默认为1000Kbps,注意波特
        REMOTE_FLAG=0,
        //电机使用不涉及远程帧，为数据帧
        EXTERN_FLAG=0,
        //电机使用不涉及扩展帧，为标准帧
        LEN,//数据长度
    };
}



#endif // ACTUATORINTERFACE_H