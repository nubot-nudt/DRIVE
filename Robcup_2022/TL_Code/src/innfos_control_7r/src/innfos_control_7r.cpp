/**
 * innfos_control
 * 机械臂
 * 吴拓昌   20230222
 **/

#include <ros/ros.h>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <cmath>
#include <iostream>
#include <thread>
#include "actuatorcontroller.h"
#include "ftsensorPerceptron.h"
#include "nubot_msgs/angle_position.h"
#include "nubot_msgs/innfosPub.h"
#include "nubot_msgs/innfosSub.h"

#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[2J\033[0;0H" // 清屏且将光标置顶

using namespace std;
using namespace chrono;

Sensor sensorL = {36, 1, 0, 0}; // 得定义成全局变量才能在别的文件中使用
Sensor sensorR = {33, 0, 0, 0};

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
        // initFTR = kw6ftsensor->initCan(sensorR.debicIndex, 1000); // 只需要初始化一次CAN ，再次初始化会被占用
        initFTL = kw6ftsensor->initCan(sensorL.debicIndex, 1000);
        /*********节点*********/
        controller = n.createTimer(ros::Duration(1.0 / 100.0),
                                   &nubot_arm_node::timeController, this, false);
        innfos_pub_ = n.advertise<nubot_msgs::innfosPub>("innfosPub", 1, true);
        innfos_sub_left = n.subscribe("innfosSub_left", 1, &nubot_arm_node::innfos_leftCallback, this);
        innfos_sub_right = n.subscribe("innfosSub_right", 1, &nubot_arm_node::innfos_rightCallback, this);
    }

    ~nubot_arm_node() // 关闭节点
    {
        cout << "Node closing！ Begin initializing" << endl;
        cout << "请先把方向盘推开,后按enter继续" << endl;
        std::this_thread::sleep_for(std::chrono::seconds(1));
        cin.get();
        if (flag_init == false)
        {
            std::this_thread::sleep_for(std::chrono::seconds(1)); // 注意：延时为灵巧手动作留出时间！！！！！！！
            for (int i = 0; i < N; i++)                           // 将各个电机设置成位置模式并归位
            {
                pController->activateActuatorMode(actuator[i], Actuator::Mode_Profile_Pos);
                pController->setProfilePositionMaxVelocity(actuator[i], Motor_PosV[i]);
                pController->setPosition(actuator[i], pos_init[i] / (360.0 / Motor_Reduction_Ratio[i]));
            }
        }
        std::this_thread::sleep_for(std::chrono::seconds(5)); // 延时5秒为电机归位预留时间
        pController->disableAllActuators();                   // 失能电机
        // if (kw6ftsensorL != NULL)
        //     delete kw6ftsensorL;
        // if (kw6ftsensorR != NULL)
        //     delete kw6ftsensorR;
    }

    void innfos_leftCallback(const nubot_msgs::innfosSub &cobra)
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
                for (int i = 0; i < N1; i++) // 将各个电机设置成位置模式
                {
                    pController->setPosition(actuator[i], cobra.firstPos[i]);
                    // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                }
                // pController->setPosition(actuator[0], cobra.firstPos[0]);
                // // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                // pController->setPosition(actuator[1], cobra.firstPos[1]);
                // // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                // pController->setPosition(actuator[2], cobra.firstPos[2]);
                // // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                // pController->setPosition(actuator[3], cobra.firstPos[3]);
                // // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                // pController->setPosition(actuator[4], cobra.firstPos[4]);
                // // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                // pController->setPosition(actuator[5], cobra.firstPos[5]);
                // // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                // pController->setPosition(actuator[6], cobra.firstPos[6]);
                // // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
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

            for (int i = 0; i < N1; i++)
            {
                firstSpeed[i] = cobra.firstSpeed[i];
                if (abs(firstSpeed[i]) < 0.01)
                    firstSpeed[i] = 0;
                cobra_SetV(actuator[i], firstSpeed[i], pos[i], LimitMax_left[i], LimitMin_left[i]);
            }
        }
        flag_left_node = 0;
        left_node_init = true;
    }

    void innfos_rightCallback(const nubot_msgs::innfosSub &cobra)
    {
        if (cobra.secondNeedMode == PMODE)
        {
            if (secondNowMode != PMODE)
            {
                for (int i = N1; i < N; i++) // 将各个电机设置成位置模式
                {
                    pController->activateActuatorMode(actuator[i], Actuator::Mode_Profile_Pos);
                    pController->setProfilePositionMaxVelocity(actuator[i], Motor_Speed[i]); // 修改电机最大速度
                }
                secondNowMode = PMODE;
            }
            if (cobra.secondPosMode == 1)
            {
                for (int i = 0; i < N2; i++) // 将各个电机设置成位置模式
                {
                    pController->setPosition(actuator[i + N1], cobra.secondPos[i]);
                    // std::this_thread::sleep_for(std::chrono::seconds(5));//延时5秒为电机归位预留时间
                }
            }
        }
        else if (cobra.secondNeedMode == VMODE)
        {
            if (secondNowMode != VMODE)
            {
                for (int i = N1; i < N; i++) // 将各个电机设置成速度模式
                    pController->activateActuatorMode(actuator[i], Actuator::Mode_Profile_Vel);
                secondNowMode = VMODE;
            }

            for (int i = 0; i < N2; i++)
            {
                secondSpeed[i] = cobra.secondSpeed[i]; // 底层指令速度为RPM
                if (abs(secondSpeed[i]) < 0.01)
                    secondSpeed[i] = 0;
                cobra_SetV(actuator[i + N1], secondSpeed[i], pos[i + N1], LimitMax_right[i], LimitMin_right[i]);
            }
        }
        flag_right_node = 0;
        right_node_init = true;
    }

    void nodeCheck() // 检查控制节点是否存在，不存在则位置模式锁定电机
    {
        if (left_node_init)
        {
            if (flag_left_node > 5)
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
                cout << "左臂节点无消息发出，设置为位置模式锁定电机" << endl;
            }
            else
            {
                flag_left_node++;
                // cout << "左臂节点计数:" << flag_left_node << endl;
            }
        }
        if (right_node_init)
        {
            if (flag_right_node > 5)
            {
                if (secondNowMode != PMODE)
                {
                    for (int i = N1; i < N; i++) // 将各个电机设置成位置模式
                    {
                        pController->activateActuatorMode(actuator[i], Actuator::Mode_Profile_Pos);
                        pController->setProfilePositionMaxVelocity(actuator[i], Motor_Speed[i]); // 修改电机最大速度
                    }
                    secondNowMode = PMODE;
                }
                cout << "右臂节点无消息发出，设置为位置模式锁定电机" << endl;
            }
            else
            {
                flag_right_node++;
                // cout << "右臂节点计数:" << flag_right_node << endl;
            }
        }
        if (firstNowMode == VMODE)
            cout << "\33[32m左臂\033[0m当前为\33[32m速度模式\033[0m" << endl;
        else
            cout << "\33[32m左臂\033[0m当前为\33[32m位置模式\033[0m" << endl;
        if (secondNowMode == VMODE)
            cout << "\33[32m右臂\033[0m当前为\33[32m速度模式\033[0m" << endl;
        else
            cout << "\33[32m右臂\033[0m当前为\33[32m位置模式\033[0m" << endl;
    }

    void sensorSelfCheck()
    {
        cout << "id" << sensorR.id << "channel" << sensorR.channel << endl;
        if ((kw6ftsensor->startFT(sensorR)))
        {
            cout << "右臂传感器自检完成,Enter继续" << endl;
            // cin.get();
        }
        else
        {
            cout << "右臂传感器未检测出数据" << endl;
            cin.get();
        }

        if ((kw6ftsensor->startFT(sensorL)))
            cout << "左臂传感器自检完成" << endl;
        else
        {
            cout << "左臂传感器未检测出数据" << endl;
            cin.get();
        }
    }

    /***************时间中断，控制部分***************/
    void timeController(const ros::TimerEvent &e)
    {
        printf(curshome); // 清屏且将光标置顶
        auto start = system_clock::now();
        // 读取数据
        ros::NodeHandle n;
        n.param<int>("cobraNum1", N1, 7);
        n.param<int>("cobraNum2", N2, 7);

        n.param<double>("kSpeedResolution", speed_resolution, 0.02);
        n.param<double>("kRotateResolution", rotate_resolution, 0.1);

        // n.getParam("firstLimitMax", LimitMax_left);
        // n.getParam("firstLimitMin", LimitMin_left);
        // n.getParam("secondLimitMax", LimitMax_second);
        // n.getParam("secondLimitMin", LimitMin_second);

        N = N1 + N2;
        All_Motor_IsOnlineAndEnable = true;
        for (int i = 0; i < N; i++)
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
            cout << "Error!!!!!!!!网线没好或电机数量设置错误!!!!!!" << endl;
            if (uIDArray.size() > 0)
            {
                int i = 0;
                for (int i = 0; i < uIDArray.size(); i++)
                {
                    actuator[i] = uIDArray.at(i).actuatorID;
                    pController->activateActuatorMode(actuator[i], Actuator::Mode_Profile_Pos);
                    pController->setProfilePositionMaxVelocity(actuator[i], Motor_Speed[i]);
                    firstNowMode = PMODE;
                    secondNowMode = PMODE;
                    NOWMODE = PMODE;
                }
            }
            enableOK = false;
            innfosPub.enableOK = enableOK;
            innfos_pub_.publish(innfosPub);
            // if(initFTR&&initFTL)
            if (initFTR) // 只需要初始化一次CAN
            {
                cout << "OPEN阶段：CAN初始化成功！开始自检" << endl;
                sensorSelfCheck();
            }
            else
            {
                cout << "CAN初始化失败！请检查设备ID与通道" << endl;
            }
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
                pController->enableVelocityFilter(actuator[i], true);
                original_vel[i] = pController->getVelocity(actuator[i], true);
                if (i < N1)
                {
                    innfosPub.firstOriginalPos[i] = original_pos[i];
                    innfosPub.firstOriginalVel[i] = (original_vel[i] + last_original_vel[i] + llast_original_vel[i]) / 3; // 平均平滑窗口
                }
                else
                {
                    innfosPub.secondOriginalPos[i - N1] = original_pos[i];
                    innfosPub.secondOriginalVel[i - N1] = (original_vel[i] + last_original_vel[i] + llast_original_vel[i]) / 3;
                }
                temperature[i] = pController->getMotorTemperature(actuator[i], true); // 獲取電機溫度
                DianLiu[i] = pController->getCurrent(actuator[i], true);
                double *FtR;
                double *FtL;
                FtL = kw6ftsensor->getFTdata(sensorL);
                for (int i = 0; i < 6; i++)
                {
                    ForceL[i] = *(FtL + i);
                    innfosPub.firstEndFS[i] = ForceL[i] * 9.81;
                }
                FtR = kw6ftsensor->getFTdata(sensorR); // getFTdata使用的是同一块内存空间，指针指向后需要立即把数据输出出来
                for (int i = 0; i < 6; i++)
                {
                    ForceR[i] = *(FtR + i);
                    innfosPub.secondEndFS[i] = ForceR[i] * 9.81;
                }

                pos[i] = round((360.0 / Motor_Reduction_Ratio[i]) * original_pos[i] * 1000) / 1000; // 除以减速比乘以360转化成目视角度
                if (flag_init)                                                                      // online后第一次读取角度数据给它存起来，且开一次节点仅存一次，方便关闭节点时，臂的归位
                    pos_init[i] = pos[i];
                llast_original_vel[i] = last_original_vel[i];
                last_original_vel[i] = original_vel[i];
                innfosPub.innfosTemperature[i] = temperature[i];
                innfosPub.innfosCurrent[i] = DianLiu[i];
            }
            flag_init = false;

            cout << "各电机角度，单位：度以及底层电机控制量速度，单位：rpm:" << endl;
            for (int i = 0; i < N; i++)
            {
                if (i < N1)
                {
                    cout << "左臂电机" << i + 1 << "[ID:" << int(actuator[i]) << "] Angle:" << fixed << setprecision(2) << setiosflags(ios::left) << setw(8) << pos[i] << "[" << setw(4) << LimitMin_left[i] << "~" << setw(3) << LimitMax_left[i] << "];";
                    cout << "速度控制指令: " << setiosflags(ios::left) << setw(6) << firstSpeed[i] << ";";
                    cout << endl;
                }

                else
                {
                    cout << "右臂电机" << i + 1 << "[ID:" << int(actuator[i]) << "] Angle:" << fixed << setprecision(2) << setiosflags(ios::left) << setw(8) << pos[i] << "[" << setw(4) << LimitMin_right[i - N1] << "~" << setw(3) << LimitMax_right[i - N1] << "];";
                    cout << "速度控制指令: " << setiosflags(ios::left) << setw(6) << secondSpeed[i - N1] << ";";
                    cout << endl;
                }
            }
            cout << "Current1:";
            for (int i = 0; i < N1; i++)
                cout << "[" << round(100 * DianLiu[i]) / 100 << "];  ";
            cout << endl;
            cout << "Temperature1:";
            for (int i = 0; i < N1; i++)
                cout << "[" << temperature[i] << "];  ";
            cout << endl;
            cout << "Current2:";
            for (int i = N1; i < N; i++)
                cout << "[" << round(100 * DianLiu[i]) / 100 << "];  ";
            cout << endl;
            cout << "Temperature2:";
            for (int i = N1; i < N; i++)
                cout << "[" << temperature[i] << "];  ";
            cout << endl;
            innfos_pub_.publish(innfosPub);
            nodeCheck();
            this_thread::sleep_for(std::chrono::microseconds(150)); // 加入CAN的帧间隔，防止数据获取失败
            auto end = system_clock::now();
            auto duration = duration_cast<microseconds>(end - start);
            cout << "Send message spent" << setprecision(7) << double(duration.count()) * microseconds::period::num / microseconds::period::den << "second" << endl;
        }
    }

private:
    ros::Timer controller;
    ros::Publisher innfos_pub_;
    ros::Subscriber innfos_sub_left;
    ros::Subscriber innfos_sub_right;

    bool All_Motor_IsOnlineAndEnable = true;

    nubot_msgs::angle_position timon_angle;
    nubot_msgs::innfosPub innfosPub;

    ftsensorPerceptron *kw6ftsensor = new ftsensorPerceptron;
    // ftsensorPerceptron *kw6ftsensorR = new ftsensorPerceptron;
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
    bool initFTR, initFTL;
    bool left_node_init = false, right_node_init = false;
    bool flag_init = true;
    double flag_left_node = 0, flag_right_node = 0;
    double pos_init[15] = {0};     // 关闭节点时，一键归位用
    double pos[15] = {0};          // 各电机位置信息 单位：°
    double original_pos[15] = {0}; // 底层电机位置信息 单位：转
    double original_vel[15] = {0};
    double last_original_vel[15] = {0};
    double llast_original_vel[15] = {0};
    double ForceR[6] = {0};
    double ForceL[6] = {0};

    int Motor_PosV[15] = {100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100}; // 位置模式时，各底层电机转速,单位：每分钟多少转

    std::vector<int> LimitMax_left = {100, 7, 160, 5, 100, 110, 105};         // 各电机最大限位 单位：° 度
    std::vector<int> LimitMin_left = {-100, -170, -20, -118, -100, -25, -60}; // 各电机最小限位 单位：° 度
    std::vector<int> LimitMax_right = {100, 170, 20, 118, 100, 25, 60};       // 各电机最大限位 单位：° 度
    std::vector<int> LimitMin_right = {-100, -7, -160, -5, -100, -110, -105}; // 各电机最小限位 单位：° 度
    // 机械臂速度： 6 * cobra_Vel/Motor_Reduction_Ratio 度/秒
    int Motor_Reduction_Ratio[15] = {36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36, 36}; // 各电机减速比

    // int Motor_Speed[15] = {200, 300, 300, 500, 500, 300, 300, 200, 300, 300, 500, 500, 300, 300}; // 底层电机最大速度，每分钟多少转  具体到角度计算： Motor_Speed/60 * 360/Motor_Reduction_Ratio = 6 * Motor_Speed/Motor_Reduction_Ratio 度/秒
    int Motor_Speed[15] = {150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150, 150}; // 底层电机最大速度，每分钟多少转  具体到角度计算： Motor_Speed/60 * 360/Motor_Reduction_Ratio = 6 * Motor_Speed/Motor_Reduction_Ratio 度/秒

    float temperature[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 电机的温度
    float DianLiu[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};     // 电机的dianliu
    int NOWMODE = 0;
    float VMODE = 1;
    float PMODE = 2;
    double firstSpeed[7] = {0, 0, 0, 0, 0, 0};
    double secondSpeed[7] = {0, 0, 0, 0, 0, 0};
    double rotate_resolution = 0.2; // 末端旋转速度的分辨率（越大速度越快）默认值
    double speed_resolution = 0.05; // 末端平移速度的分辨率（越大速度越快）默认值
    int N1, N2, N = N1 + N2;
    int i;
    bool enableOK = true;
    float firstNowMode = 0;
    float secondNowMode = 0;
    // ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑参数调节↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
};

int main(int argc, char *argv[])
{
    /*******************************ROS*****************************/
    // sensorL[0].id = 22;        // 传感器设备ID号
    // sensorL[0].channel = 0;    // CAN分析设备通道号
    // sensorL[0].debicIndex = 0; // CAN分析设备ID号
    // sensorL[0].errornum = 0;
    // sensorR[0].id = 21;
    // sensorR[0].channel = 1;
    // sensorR[0].debicIndex = 0;
    // sensorR[0].errornum = 0;
    ros::init(argc, argv, "innfos_control_7r");
    nubot_arm_node arm;
    ros::NodeHandle n;
    //    n.param<int>("cobraNum1",N1,6);
    //    n.param<int>("cobraNum2",N2,6);

    ros::spin();
    return 0;
}
