/**
 * cc_7rl3
 * 机械臂
 * 吴拓昌   20230222
 **/

#include <ros/ros.h>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include "actuatorcontroller.h"
#include "/home/luo/Coppeliasim_Carla/Robcup_2022/ROS_Code/devel/.private/nubot_msgs/include/nubot_msgs/angle_position.h"
#include "/home/luo/Coppeliasim_Carla/Robcup_2022/ROS_Code/devel/.private/nubot_msgs/include/nubot_msgs/innfosPub.h"
#include "/home/luo/Coppeliasim_Carla/Robcup_2022/ROS_Code/devel/.private/nubot_msgs/include/nubot_msgs/innfosSub.h"

#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

using namespace std;

ActuatorController *pController = ActuatorController::initController(); // 电机初始化 在最开始必须申明出 pController 否则后面类中函数有关pController的地方都会报错
Actuator::ErrorsDefine ec;
std::vector<ActuatorController::UnifiedID> uIDArray = pController->lookupActuators(ec);                   // 查找可用设备号
int a = pController->enableActuatorInBatch(uIDArray);                                                     // 使能电机
unsigned char actuator[15] = {'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'}; // 先申明并任意赋值设备号，防止后续代码报错

/** ↓↓↓↓↓↓↓↓↓↓cobra_SetV函数↓↓↓↓↓↓↓↓↓↓
* @brief 设置速度
* @param id 执行器id
* @param vel 目标速度，单位是转/每分钟
* @param pos 当前位置，单位是°
* @param LMax,LMin 最大最小位置，单位是°

**/

void cobra_SetV(uint8_t id, double vel, double pos, int LMax, int LMin)
{
    if (pos >= LMax)
    {
        if (vel > 0)
            pController->setVelocity(id, 0);
        else
            pController->setVelocity(id, vel);
    }
    else if (pos <= LMin)
    {
        if (vel > 0)
            pController->setVelocity(id, vel);
        else
            pController->setVelocity(id, 0);
    }
    else
    {
        pController->setVelocity(id, vel);
    }
}

/** ↓↓↓↓↓↓↓↓↓↓cobra_isOnlineAndEnable函数↓↓↓↓↓↓↓↓↓↓
 * @brief 检查设备是否掉线或失能  设备有电且失能返回true，否则返回false
 * @param id 执行器id
 **/
bool cobra_isOnlineAndEnable(uint8_t id)
{
    if (pController->isOnline(id))
    {
        if (pController->isEnable(id))
            return true;
    }
    else
        return false;
}

class nubot_arm_node
{
public:
    nubot_arm_node()
    {
        ros::NodeHandle n;
        /*********节点*********/
        controller = n.createTimer(ros::Duration(1.0 / 100.0),
                                   &nubot_arm_node::timeController, this, false);
        cobra_angle_pub_ = n.advertise<nubot_msgs::angle_position>("timon_angle", 1);
        innfos_pub_ = n.advertise<nubot_msgs::innfosPub>("innfosPub", 1, true);
        innfos_sub_ = n.subscribe("innfosSub", 1, &nubot_arm_node::innfosCallback, this);
    }

    ~nubot_arm_node() // 关闭节点
    {
        std::cout << "Node closing！ Begin initializing" << std::endl;
        std::this_thread::sleep_for(std::chrono::seconds(5)); // 延时5秒为电机归位预留时间
        for (int i = 0; i < N1; i++)                          // 将各个电机设置成位置模式并归位
        {
            pController->activateActuatorMode(actuator[i], Actuator::Mode_Profile_Pos);
            pController->setProfilePositionMaxVelocity(actuator[i], Motor_PosV[i]);
            pController->setPosition(actuator[i], pos_init[i] / (360.0 / Motor_Reduction_Ratio[i]));
        }
        std::this_thread::sleep_for(std::chrono::seconds(5)); // 延时5秒为电机归位预留时间
        pController->disableAllActuators();                   // 失能电机
    }

    void innfosCallback(const nubot_msgs::innfosSub &cobra)
    {
        if (cobra.firstNeedMode == PMODE)
        {
            if (firstNowMode != PMODE)
            {
                for (int i = 0; i < N1; i++) // 将各个电机设置成位置模式
                {
                    pController->activateActuatorMode(actuator[i], Actuator::Mode_Profile_Pos);
                    pController->setProfilePositionMaxVelocity(actuator[i], Motor_Speed[i]); // 修改电机最大速度
                }
                firstNowMode = PMODE;
            }
            if (cobra.firstPosMode == 1)
            {
                pController->setPosition(actuator[0], cobra.firstPos[0]);
                // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                pController->setPosition(actuator[1], cobra.firstPos[1]);
                // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                pController->setPosition(actuator[2], cobra.firstPos[2]);
                // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                pController->setPosition(actuator[3], cobra.firstPos[3]);
                // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                pController->setPosition(actuator[4], cobra.firstPos[4]);
                // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                pController->setPosition(actuator[5], cobra.firstPos[5]);
                // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                pController->setPosition(actuator[6], cobra.firstPos[6]);
            }
        }
        else if (cobra.firstNeedMode == VMODE)
        {
            if (firstNowMode != VMODE)
            {
                for (int i = 0; i < N1; i++) // 将各个电机设置成速度模式
                    pController->activateActuatorMode(actuator[i], Actuator::Mode_Profile_Vel);
                firstNowMode = VMODE;
            }
            for (i = 0; i < N1; i++)
            {
                firstSpeed[i] = cobra.firstSpeed[i];
                if (abs(firstSpeed[i]) < 0.01)
                    firstSpeed[i] = 0;
                cobra_SetV(actuator[i], firstSpeed[i], pos[i], LimitMax[i], LimitMin[i]);
            }
        }
    }

    /***************时间中断，控制部分***************/
    void timeController(const ros::TimerEvent &e)
    {
        // 读取数据
        ros::NodeHandle n;
        n.param<int>("cobraNum1", N1, 7);
        n.param<int>("cobraNum2", N2, 0);
        // 机械臂参数
        n.param<double>("firstCobra_a3", a3, 0.28);
        n.param<double>("firstCobra_a6", a6, 0.100);
        n.param<double>("firstCobra_d1", d1, 0.0612);
        n.param<double>("firstCobra_d4", d4, 0.265);
        n.param<double>("firstCobra_d6", d6, 0.035);
        N = N1 + N2;
        All_Motor_IsOnlineAndEnable = true;
        for (int i = 0; i < 7; i++) // 有几个电机串联在CAN总线上  i就小于几  待改地方！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！虽然只使用其中6个进行逆解，但此处7R，有7个电机，所以为7
        {
            All_Motor_IsOnlineAndEnable = All_Motor_IsOnlineAndEnable & cobra_isOnlineAndEnable(actuator[i]);
            if (!All_Motor_IsOnlineAndEnable)
                break;
        }

        /*如果有设备掉线或者没有失能*/ // ↓↓↓↓↓↓↓↓↓↓↓↓↓初始化↓↓↓↓↓↓↓↓↓↓↓↓↓

        if (!All_Motor_IsOnlineAndEnable)
        {
            ActuatorController *pController = ActuatorController::initController();
            Actuator::ErrorsDefine ec;
            std::vector<ActuatorController::UnifiedID> uIDArray = pController->lookupActuators(ec);
            pController->enableActuatorInBatch(uIDArray);
            if (uIDArray.size() > 0)
            {
                int i = 0;
                for (int i = 0; i < uIDArray.size(); i++)
                {
                    actuator[i] = uIDArray.at(i).actuatorID;
                    pController->activateActuatorMode(actuator[i], Actuator::Mode_Profile_Vel);
                    firstNowMode = VMODE;
                    NOWMODE = VMODE;
                }
            }
            std::cout << "Error!!!!!!!!网线没好!!!!!!" << std::endl;
            enableOK = false;
            innfosPub.enableOK = enableOK;
            innfos_pub_.publish(innfosPub);
        }
        /*所有电机都在线且使能成功,则开始控制*/
        else
        {
            enableOK = true;
            innfosPub.enableOK = enableOK;
            innfos_pub_.publish(innfosPub);
            /*************算出机械臂各关节位置并输出到屏幕********/
            // nubot_msgs::innfosPub innfosPub;
            for (int i = 0; i < N; i++)
            {
                original_pos[i] = pController->getPosition(actuator[i], true); // 得到底层电机位置 单位：转
                if (i < N1)
                {
                    innfosPub.firstOriginalPos[i] = original_pos[i];
                    pos[i] = round((360.0 / Motor_Reduction_Ratio[i]) * original_pos[i]); // 除以减速比乘以360转化成目视角度
                    if (flag_init)                                                        // online后第一次读取角度数据给它存起来，且开一次节点仅存一次，方便关闭节点时，臂的归位
                        pos_init[i] = pos[i];
                }
                else
                    innfosPub.secondOriginalPos[i - N1] = original_pos[i];
                temperature[i] = pController->getMotorTemperature(actuator[i], true); // 獲取電機溫度
                DianLiu[i] = pController->getCurrent(actuator[i], true);
                innfosPub.innfosTemperature[i] = temperature[i];
                innfosPub.innfosCurrent[i] = DianLiu[i];
            }
            flag_init = false;

            std::cout << "各电机角度，单位：度以及底层电机控制量速度，单位：rpm:" << std::endl;
            for (int i = 0; i < N; i++)
            {
                std::cout << "电机" << i + 1 << "[ID:" << int(actuator[i]) << "] Angle:" << fixed << setprecision(2) << setiosflags(ios::left) << setw(7) << pos[i] << "[" << setw(4) << LimitMin[i] << "~" << setw(3) << LimitMax[i] << "];";
                std::cout << "速度控制指令: " << setiosflags(ios::left) << setw(6) << firstSpeed[i] << ";";
            }
            cout << "Current:";
            for (int i = 0; i < N; i++)
                cout << "[" << round(100 * DianLiu[i]) / 100 << "];";
            cout << endl;
            cout << "Temperature:";
            for (int i = 0; i < N; i++)
                cout << temperature[i] << ";";
            cout << endl;

            innfos_pub_.publish(innfosPub);
        }
        std::cout << "  a3:  " << a3 << "  a6:  " << a6 << std::endl;
        std::cout << "  d1:  " << d1 << "  d4:  " << d4 << "  d6:  " << d6 << std::endl;
        // for (int i = 0; i < N; i++)
        // {
        //     timon_angle.arm[i] = pos[i];
        //     std::cout  << timon_angle.arm[i]<<";";
        // }
        // cobra_angle_pub_.publish(timon_angle); //发布爪子张开信息

        // cout << cobra3_now_mode << endl;
        for (int i = 0; i <= 20; i++)
        {
            printf(cursup);
            printf(cursclean);
        }
    }

private:
    ros::Timer controller;
    ros::Publisher innfos_pub_;
    ros::Subscriber innfos_sub_;
    ros::Publisher cobra_angle_pub_;
    bool All_Motor_IsOnlineAndEnable = true;
    // 机械臂参数
    double a2, a3, a4, d5, d1, d4, d6, a6;
    double theta[9] = {0}; // 末端控制状态角度 ，下标从1开始，theta[0]无用。单位弧度。

    int flagX = 0;
    int flagY = 0;
    int flagA = 0;
    int flagB = 0;
    int flagAB = 0;
    int flagAX = 0;
    int flagXY = 0;
    int flagBY = 0; // 八种位置模式组合键
    nubot_msgs::angle_position timon_angle;
    nubot_msgs::innfosPub innfosPub;
    /**
    ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓参数调节↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    各数组序号
    * 0 大臂旋转
    * 1 大臂前后
    * 2 小臂上下
    * 3 爪的上下
    * 4 爪的左右
    * 5 爪的旋转
    **/
    bool flag_init = true;
    double pos_init[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};     // 一键归位用
    double pos[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};          // 各电机位置信息 单位：°
    double original_pos[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 底层电机位置信息 单位：转
    //    double pos2theta[6] = {0, -180, -90, 0, 90, 90}; //当前位置到末端控制状态角转换差值
    // int pos2theta_sign[6] = {-1,1,-1,1,-1,-1};//转换差值后取正or取负

    int Motor_PosV[6] = {150, 150, 300, 350, 200, 200}; // 位置模式时，各底层电机转速,单位：每分钟多少转

    int LimitMax[6] = {180, 85, 30, 300, 120, 60};         // 各电机最大限位 单位：° 度
    int LimitMin[6] = {-180, -42, -110, -300, -120, -145}; // 各电机最小限位 单位：° 度
    // 机械臂速度： 6 * cobra_Vel/Motor_Reduction_Ratio 度/秒
    int Motor_Reduction_Ratio[6] = {36, 36, 36, 36, 36, 36}; // 各电机减速比

    int Motor_Speed[6] = {200, 300, 300, 500, 500, 300}; // 底层电机最大速度，每分钟多少转  具体到角度计算： Motor_Speed/60 * 360/Motor_Reduction_Ratio = 6 * Motor_Speed/Motor_Reduction_Ratio 度/秒

    float temperature[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 电机的温度
    float DianLiu[14] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};     // 电机的dianliu
    int NOWMODE = 0;
    float VMODE = 1;
    float PMODE = 2;
    double firstSpeed[6] = {0, 0, 0, 0, 0, 0};
    double secondSpeed[6] = {0, 0, 0, 0, 0, 0};
    int N1, N2, N = N1 + N2;
    int i;
    bool enableOK = true;
    float firstNowMode = 3;
    float secondNowMode = 3;
    // ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑参数调节↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
};

int main(int argc, char *argv[])
{
    /*******************************ROS*****************************/
    ros::init(argc, argv, "innfos_control");
    nubot_arm_node arm;
    ros::NodeHandle n;
    //    n.param<int>("cobraNum1",N1,6);
    //    n.param<int>("cobraNum2",N2,6);

    ros::spin();
    return 0;
}
