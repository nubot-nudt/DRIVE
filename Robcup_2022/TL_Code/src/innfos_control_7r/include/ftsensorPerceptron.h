#ifndef FTSENSORPERCEPTRON_H
#define FTSENSORPERCEPTRON_H
#include "canthread.h"
#include "ftsensorInterface.h"
#include "geometry_msgs/Wrench.h"
#include "std_msgs/Int32.h"

using namespace kwFTsensor;

struct Sensor
{
    UINT id;         // 收发信地址
    UINT channel;    // CAN通道号
    UINT debicIndex; // CAN设备号
    UINT errornum;
};

class ftsensorPerceptron : public CANThread
{
private:
    CANThread ft;
    CANThread *canthread = &ft;

    /* data */
public:
    ftsensorPerceptron();
    ~ftsensorPerceptron();

    bool initCan(UINT debicIndex, UINT baudrate);
    bool closeCan(UINT debicIndex);

    void sendFTdata_always(Sensor sensor) const;
    /**
     * @brief 一直发送获取六维传感器数值
     * @param id 执行器CANid
     * @param id 执行器CAN channel
     * @return 当前力与力矩，力单位为N，力矩为NM
     **/
    void StopFTdata_always(Sensor sensor) const;
    /**
     * @brief  停止发送获取六维传感器数值
     * @param id 执行器CANid
     * @param id 执行器CAN channel
     * @return 当前力与力矩，力单位为N，力矩为NM
     **/
    double *getFTdata(Sensor sensor);
    /**
     * @brief 发送一次六维传感器数值
     * @param id 执行器CANid
     * @param id 执行器CAN channel
     * @return 当前力与力矩，力单位为N，力矩为NM
     **/
    bool startFT(Sensor sensor) const;
    /**
     * @brief 传感器自检，有无正常启动
     * @param id 执行器CANid
     * @param id 执行器CAN channel
     * @return 自检成功返回1
     **/
    double (*getFTdata_batch(Sensor *sensor, INT batch))[6];
    /**
     * @brief 获取一次多个六维传感器的全部数值FxFyFzMxMyMz
     * @param id 执行器CANid
     * @param id 执行器CAN channel
     * @return 当前力与力矩，力单位为N，力矩为NM
     **/
    // double getForceData(UINT ID,UINT channel)const;
    // /**
    //  * @brief 获取当前传感器力数值
    //  * @param id 执行器CANid
    //  * @return 当前位置，单位是电机转子旋转速度 rpm
    // **/
    //    double getTorqueData(UINT ID,UINT channel)const;
    // /**
    //  * @brief 获取当前传感器力矩数值
    //  * @param id 执行器CANid
    //  * @return 当前力矩单位是NM
    // **/
    void setFTsensorID(UINT debicIndex, UINT ID, UINT channel, UINT Val) const;
    /**
     * @brief 设置波特率
     * @param id 执行器CANid数组
     * @param id 执行器CAN channel
     * @return 当前位置，单位是转数
     **/
    void setHomingFTdata(UINT debicIndex, UINT ID, UINT channel, double Val) const;
    /**
     * @brief 使得传感器归0位
     * @param id 执行器CANid数组
     * @param id 执行器CAN channel
     * @return 当前偏置
     **/
    void setBaudrate(UINT debicIndex, UINT ID, UINT channel, UINT Val) const;
    /**
     * @brief 设置波特率
     * @param id 执行器CANid数组
     * @param channel 执行器CAN channel
     * @param  Val 波特率数值
     * @return 无
     **/
};

#endif // FTSENSORPERCEPTRON_H
