/**
 * test_tgcontrol
 * 接收joy指令,控制电机速度与方向
 * 吴拓昌   20230306
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
#include "sensor_msgs/Joy.h"

#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

#define JOINT_MODE 1
#define END_MODE 2

using namespace std;

ActuatorController *pController = ActuatorController::initController(); // 电机初始化 在最开始必须申明出 pController 否则后面类中函数有关pController的地方都会报错
Actuator::ErrorsDefine ec;
std::vector<ActuatorController::UnifiedID> uIDArray = pController->lookupActuators(ec);                   // 查找可用设备号
int a = pController->enableActuatorInBatch(uIDArray);                                                     // 使能电机
unsigned char actuator[15] = {'0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0', '0'}; // 先申明并任意赋值设备号，防止后续代码报错

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

class nubot_arm_node
{
public:
    nubot_arm_node()
    {
        ros::NodeHandle n;
        /*********节点*********/
        joy_sub_ = n.subscribe("/joy", 1, &nubot_arm_node::joyCallback, this);
        controller = n.createTimer(ros::Duration(1.0 / 100.0), &nubot_arm_node::timeController, this, false);
    }

    ~nubot_arm_node() // 关闭节点
    {
        std::cout << "Node closing！ Begin initializing" << std::endl;

        pController->disableAllActuators(); // 失能电机
    }
    /*********↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓肩关节三电机解算↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*********/
    void CartesianToJoint(bool mode)
    {
        // 求出姿态
        nx = cos(theta[2] + theta[3]) * sin(theta[1]);

        ny = sin(theta[2] + theta[3]);

        nz = cos(theta[2] + theta[3]) * cos(theta[1]);

        ox = -sin(theta[2] + theta[3]) * sin(theta[1]);

        oy = cos(theta[2] + theta[3]);

        oz = -sin(theta[2] + theta[3]) * cos(theta[1]);

        ax = -cos(theta[1]);

        ay = 0;

        az = sin(theta[1]);

        px = -d3 * cos(theta[1]);

        py = 0;

        pz = d3 * sin(theta[2]);

        // 3轴肩关节下,末端相对基座运动仅有3个自由度dpitch dyaw droll，但dx,dy,dz也可以存在，会受到约束，持续施加dxdydz最终使得臂沿基座坐标系的XYZ轴伸直展开
        // 可通过变换矩阵B把基座的运动映射到其他坐标系 eg：A*T=B B标系下的微分运动DDB  该运动在A坐标系下映射为DDA，则DDB = T^-1*DDA*T,(或将该公式理解为A坐标系下微分运动DDA分解到B坐标系下的DDB);

        if (mode) // 基座坐标系   绕末端z轴rz=roll      绕末端y轴 ry yaw     绕末端X轴 rx pitch,
        {
            // std::cout << "基座";
            dnx = dpitch * nz - dyaw * ny;
            dny = dyaw * nx - droll * nz;
            dnz = droll * ny - dpitch * nx;

            dox = dpitch * oz - dyaw * oy;
            doy = dyaw * ox - droll * oz;
            doz = droll * oy - dpitch * ox;

            dax = az * dpitch - ay * dyaw;
            day = ax * dyaw - az * droll;
            daz = ay * droll - ax * dpitch;

            dpx = dx + dpitch * pz - dyaw * py;
            dpy = dy - droll * pz + dyaw * px;
            dpz = dz - dpitch * px + droll * py;

            /*           dnx = ((pow(2,0.5)*dpitch)/2 - (pow(2,0.5)*dyaw)/2) * nz - (-droll) * ny;
                      dny = (-droll) * nx - ((pow(2,0.5)*dpitch)/2 + (pow(2,0.5)*dyaw)/2) * nz;
                      dnz = ((pow(2,0.5)*dpitch)/2 + (pow(2,0.5)*dyaw)/2) * ny - ((pow(2,0.5)*dpitch)/2 - (pow(2,0.5)*dyaw)/2) * nx;

                      dox = ((pow(2,0.5)*dpitch)/2 - (pow(2,0.5)*dyaw)/2) * oz - (-droll) * oy;
                      doy = (-droll) * ox - ((pow(2,0.5)*dpitch)/2 + (pow(2,0.5)*dyaw)/2) * oz;
                      doz = ((pow(2,0.5)*dpitch)/2 + (pow(2,0.5)*dyaw)/2) * oy - ((pow(2,0.5)*dpitch)/2 - (pow(2,0.5)*dyaw)/2) * ox;

                      dax = az * ((pow(2,0.5)*dpitch)/2 - (pow(2,0.5)*dyaw)/2) - ay * (-droll);
                      day = ax * (-droll) - az * ((pow(2,0.5)*dpitch)/2 + (pow(2,0.5)*dyaw)/2);
                      daz = ay * ((pow(2,0.5)*dpitch)/2 + (pow(2,0.5)*dyaw)/2) - ax * ((pow(2,0.5)*dpitch)/2 - (pow(2,0.5)*dyaw)/2);

                      dpx = (pow(2,0.5)*dx)/2 - (pow(2,0.5)*dy)/2 + ((pow(2,0.5)*dpitch)/2 - (pow(2,0.5)*dyaw)/2) * pz - (-droll) * py;
                      dpy = -dz - ((pow(2,0.5)*dpitch)/2 + (pow(2,0.5)*dyaw)/2) * pz + (-droll) * px;
                      dpz = (pow(2,0.5)*dx)/2 + (pow(2,0.5)*dy)/2 - ((pow(2,0.5)*dpitch)/2 - (pow(2,0.5)*dyaw)/2) * px + ((pow(2,0.5)*dpitch)/2 + (pow(2,0.5)*dyaw)/2) * py; */
        }
        // 3轴肩关节下，末端相对末端运动只有3个自由度，dx dy droll
        if (!mode) // 末端坐标系  视作T06*TDD，其中TDD与DD拥有相同的形式，但含义不同
        {
            // std::cout << "末端";
            dnx = droll * ox - ax * dyaw;
            dny = droll * oy - ay * dyaw;
            dnz = droll * oz - az * dyaw;

            dox = ax * dpitch - droll * nx;
            doy = ay * dpitch - droll * ny;
            doz = az * dpitch - droll * nz;

            dax = dyaw * nx - dpitch * ox;
            day = dyaw * ny - dpitch * oy;
            daz = dyaw * nz - dpitch * oz;

            dpx = ax * dz + dx * nx + dy * ox;
            dpy = ay * dz + dx * ny + dy * oy;
            dpz = az * dz + dx * nz + dy * oz;
        }
        // 解算速度,单位弧度每秒
        // dtheta[1] = -(dy * cos(theta[2] + theta[3]) + dx * sin(theta[2] + theta[3])) / (d3 * sin(theta[1]) * (pow(cos(theta[2] + theta[3]), 2) + pow(sin(theta[2] + theta[3]), 2)));

        // dtheta[2] = (dx * cos(theta[2] + theta[3]) - dy * sin(theta[2] + theta[3])) / (d3 * (pow(cos(theta[2] + theta[3]), 2) + pow(sin(theta[2] + theta[3]), 2)));

        // dtheta[3] = -(d3 * droll * sin(theta[1]) * pow(cos(theta[2] + theta[3]), 2) + dy * cos(theta[1]) * cos(theta[2] + theta[3]) + d3 * droll * sin(theta[1]) * pow(sin(theta[2] + theta[3]), 2) + dx * cos(theta[1]) * sin(theta[2] + theta[3])) / (d3 * sin(theta[1]) * (pow(cos(theta[2] + theta[3]), 2) + pow(sin(theta[2] + theta[3]), 2)));

        dtheta[1] = -(dnz * pow(cos(theta[1]), 2) * cos(theta[3]) * sin(theta[2]) - dnz * sin(theta[2] + theta[3]) * pow(cos(theta[3]), 2) - dnz * sin(theta[2] + theta[3]) * pow(cos(theta[1]), 2) * pow(sin(theta[2]), 2) + dny * sin(theta[2] + theta[3]) * cos(theta[1]) * cos(theta[2]) * sin(theta[2]) + dnx * cos(theta[1]) * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) - dnx * sin(theta[2] + theta[3]) * cos(theta[1]) * sin(theta[1]) * pow(sin(theta[2]), 2) + dnz * pow(nz, 2) * pow(cos(theta[1]), 2) * cos(theta[2]) * sin(theta[3]) + dny * oz * sin(theta[2] + theta[3]) * cos(theta[2]) * cos(theta[3]) + dnz * ny * sin(theta[2] + theta[3]) * cos(theta[3]) * sin(theta[2]) + dny * nz * sin(theta[2] + theta[3]) * cos(theta[2]) * sin(theta[3]) + dnx * pow(nz, 2) * cos(theta[1]) * cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + dnz * ny * oz * cos(theta[1]) * cos(theta[3]) * sin(theta[2]) + dnz * ny * nz * cos(theta[1]) * sin(theta[2]) * sin(theta[3]) + dnx * ny * oz * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) + dnx * ny * nz * sin(theta[1]) * sin(theta[2]) * sin(theta[3]) + dnz * nz * sin(theta[2] + theta[3]) * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) + dnz * nx * sin(theta[2] + theta[3]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) - dnz * oz * sin(theta[2] + theta[3]) * cos(theta[1]) * cos(theta[3]) * sin(theta[2]) + dnz * nz * oz * pow(cos(theta[1]), 2) * cos(theta[2]) * cos(theta[3]) - dnz * nz * sin(theta[2] + theta[3]) * cos(theta[1]) * sin(theta[2]) * sin(theta[3]) - dnx * oz * sin(theta[2] + theta[3]) * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) + dnx * nx * oz * cos(theta[2]) * cos(theta[3]) * pow(sin(theta[1]), 2) - dnx * nz * sin(theta[2] + theta[3]) * sin(theta[1]) * sin(theta[2]) * sin(theta[3]) + dnx * nx * nz * cos(theta[2]) * pow(sin(theta[1]), 2) * sin(theta[3]) + dnz * nx * oz * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) + dnx * nz * oz * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) + dnz * nx * nz * cos(theta[1]) * cos(theta[2]) * sin(theta[1]) * sin(theta[3])) / (nx * sin(theta[2] + theta[3]) * pow(cos(theta[2]), 2) * pow(sin(theta[1]), 2) - nx * sin(theta[2] + theta[3]) * pow(cos(theta[1]), 2) * pow(sin(theta[2]), 2) - sin(theta[2] + theta[3]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) + nx * pow(cos(theta[1]), 2) * cos(theta[3]) * sin(theta[2]) + ny * sin(theta[2] + theta[3]) * cos(theta[2]) * sin(theta[1]) * sin(theta[2]) - nz * cos(theta[1]) * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) - ny * pow(nz, 2) * sin(theta[1]) * sin(theta[2]) * sin(theta[3]) + nz * sin(theta[2] + theta[3]) * cos(theta[1]) * pow(cos(theta[2]), 2) * sin(theta[1]) + nz * sin(theta[2] + theta[3]) * cos(theta[1]) * sin(theta[1]) * pow(sin(theta[2]), 2) + nx * pow(nz, 2) * pow(cos(theta[1]), 2) * cos(theta[2]) * sin(theta[3]) + pow(nz, 2) * sin(theta[2] + theta[3]) * sin(theta[1]) * sin(theta[2]) * sin(theta[3]) - nx * pow(nz, 2) * cos(theta[2]) * pow(sin(theta[1]), 2) * sin(theta[3]) - pow(nz, 3) * cos(theta[1]) * cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + pow(nx, 2) * oz * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) - pow(nz, 2) * oz * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) + pow(nx, 2) * nz * cos(theta[1]) * cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + nx * ny * oz * cos(theta[1]) * cos(theta[3]) * sin(theta[2]) + nx * ny * nz * cos(theta[1]) * sin(theta[2]) * sin(theta[3]) - ny * nz * oz * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) - nx * oz * sin(theta[2] + theta[3]) * cos(theta[1]) * cos(theta[3]) * sin(theta[2]) + nx * nz * oz * pow(cos(theta[1]), 2) * cos(theta[2]) * cos(theta[3]) - nx * nz * sin(theta[2] + theta[3]) * cos(theta[1]) * sin(theta[2]) * sin(theta[3]) + nz * oz * sin(theta[2] + theta[3]) * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) - nx * nz * oz * cos(theta[2]) * cos(theta[3]) * pow(sin(theta[1]), 2));

        dtheta[2] = (dnz * nz * pow(cos(theta[3]), 2) * sin(theta[1]) - dnz * nx * cos(theta[1]) * pow(cos(theta[3]), 2) + dny * sin(theta[2] + theta[3]) * pow(cos(theta[2]), 2) * sin(theta[1]) + dnx * cos(theta[2]) * cos(theta[3]) * pow(sin(theta[1]), 2) + dnz * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) - dny * pow(nz, 2) * cos(theta[2]) * sin(theta[1]) * sin(theta[3]) - dnx * sin(theta[2] + theta[3]) * cos(theta[2]) * pow(sin(theta[1]), 2) * sin(theta[2]) + dny * nx * oz * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) + dny * nx * nz * cos(theta[1]) * cos(theta[2]) * sin(theta[3]) - dny * nz * oz * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) + dnz * nx * sin(theta[2] + theta[3]) * cos(theta[1]) * cos(theta[3]) * sin(theta[2]) - dnz * nz * sin(theta[2] + theta[3]) * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) - dnz * sin(theta[2] + theta[3]) * cos(theta[1]) * cos(theta[2]) * sin(theta[1]) * sin(theta[2])) / (nx * sin(theta[2] + theta[3]) * pow(cos(theta[2]), 2) * pow(sin(theta[1]), 2) - nx * sin(theta[2] + theta[3]) * pow(cos(theta[1]), 2) * pow(sin(theta[2]), 2) - sin(theta[2] + theta[3]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) + nx * pow(cos(theta[1]), 2) * cos(theta[3]) * sin(theta[2]) + ny * sin(theta[2] + theta[3]) * cos(theta[2]) * sin(theta[1]) * sin(theta[2]) - nz * cos(theta[1]) * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) - ny * pow(nz, 2) * sin(theta[1]) * sin(theta[2]) * sin(theta[3]) + nz * sin(theta[2] + theta[3]) * cos(theta[1]) * pow(cos(theta[2]), 2) * sin(theta[1]) + nz * sin(theta[2] + theta[3]) * cos(theta[1]) * sin(theta[1]) * pow(sin(theta[2]), 2) + nx * pow(nz, 2) * pow(cos(theta[1]), 2) * cos(theta[2]) * sin(theta[3]) + pow(nz, 2) * sin(theta[2] + theta[3]) * sin(theta[1]) * sin(theta[2]) * sin(theta[3]) - nx * pow(nz, 2) * cos(theta[2]) * pow(sin(theta[1]), 2) * sin(theta[3]) - pow(nz, 3) * cos(theta[1]) * cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + pow(nx, 2) * oz * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) - pow(nz, 2) * oz * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) + pow(nx, 2) * nz * cos(theta[1]) * cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + nx * ny * oz * cos(theta[1]) * cos(theta[3]) * sin(theta[2]) + nx * ny * nz * cos(theta[1]) * sin(theta[2]) * sin(theta[3]) - ny * nz * oz * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) - nx * oz * sin(theta[2] + theta[3]) * cos(theta[1]) * cos(theta[3]) * sin(theta[2]) + nx * nz * oz * pow(cos(theta[1]), 2) * cos(theta[2]) * cos(theta[3]) - nx * nz * sin(theta[2] + theta[3]) * cos(theta[1]) * sin(theta[2]) * sin(theta[3]) + nz * oz * sin(theta[2] + theta[3]) * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) - nx * nz * oz * cos(theta[2]) * cos(theta[3]) * pow(sin(theta[1]), 2));

        dtheta[3] = -(dny * sin(theta[2] + theta[3]) * pow(cos(theta[2]), 2) * sin(theta[1]) + dnx * nx * pow(cos(theta[2]), 2) * pow(sin(theta[1]), 3) - dny * nx * pow(cos(theta[1]), 2) * cos(theta[2]) * sin(theta[2]) + dnx * ny * cos(theta[2]) * pow(sin(theta[1]), 2) * sin(theta[2]) - dnx * sin(theta[2] + theta[3]) * cos(theta[2]) * pow(sin(theta[1]), 2) * sin(theta[2]) + dnz * nx * cos(theta[1]) * pow(cos(theta[2]), 2) * pow(sin(theta[1]), 2) + dnx * nz * cos(theta[1]) * pow(cos(theta[2]), 2) * pow(sin(theta[1]), 2) + dnz * nz * pow(cos(theta[1]), 2) * pow(cos(theta[2]), 2) * sin(theta[1]) - dnz * pow(nx, 2) * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) + dnz * pow(nz, 2) * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) - dnz * nx * ny * cos(theta[1]) * cos(theta[3]) * sin(theta[2]) + dnz * ny * nz * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) + dnz * nx * sin(theta[2] + theta[3]) * cos(theta[1]) * cos(theta[3]) * sin(theta[2]) - dnz * nx * nz * pow(cos(theta[1]), 2) * cos(theta[2]) * cos(theta[3]) - dnz * nz * sin(theta[2] + theta[3]) * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) + dnz * nx * nz * cos(theta[2]) * cos(theta[3]) * pow(sin(theta[1]), 2) + dnz * ny * cos(theta[1]) * cos(theta[2]) * sin(theta[1]) * sin(theta[2]) + dny * nz * cos(theta[1]) * cos(theta[2]) * sin(theta[1]) * sin(theta[2]) - dnz * sin(theta[2] + theta[3]) * cos(theta[1]) * cos(theta[2]) * sin(theta[1]) * sin(theta[2])) / (nx * sin(theta[2] + theta[3]) * pow(cos(theta[2]), 2) * pow(sin(theta[1]), 2) - nx * sin(theta[2] + theta[3]) * pow(cos(theta[1]), 2) * pow(sin(theta[2]), 2) - sin(theta[2] + theta[3]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) + nx * pow(cos(theta[1]), 2) * cos(theta[3]) * sin(theta[2]) + ny * sin(theta[2] + theta[3]) * cos(theta[2]) * sin(theta[1]) * sin(theta[2]) - nz * cos(theta[1]) * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) - ny * pow(nz, 2) * sin(theta[1]) * sin(theta[2]) * sin(theta[3]) + nz * sin(theta[2] + theta[3]) * cos(theta[1]) * pow(cos(theta[2]), 2) * sin(theta[1]) + nz * sin(theta[2] + theta[3]) * cos(theta[1]) * sin(theta[1]) * pow(sin(theta[2]), 2) + nx * pow(nz, 2) * pow(cos(theta[1]), 2) * cos(theta[2]) * sin(theta[3]) + pow(nz, 2) * sin(theta[2] + theta[3]) * sin(theta[1]) * sin(theta[2]) * sin(theta[3]) - nx * pow(nz, 2) * cos(theta[2]) * pow(sin(theta[1]), 2) * sin(theta[3]) - pow(nz, 3) * cos(theta[1]) * cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + pow(nx, 2) * oz * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) - pow(nz, 2) * oz * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) + pow(nx, 2) * nz * cos(theta[1]) * cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + nx * ny * oz * cos(theta[1]) * cos(theta[3]) * sin(theta[2]) + nx * ny * nz * cos(theta[1]) * sin(theta[2]) * sin(theta[3]) - ny * nz * oz * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) - nx * oz * sin(theta[2] + theta[3]) * cos(theta[1]) * cos(theta[3]) * sin(theta[2]) + nx * nz * oz * pow(cos(theta[1]), 2) * cos(theta[2]) * cos(theta[3]) - nx * nz * sin(theta[2] + theta[3]) * cos(theta[1]) * sin(theta[2]) * sin(theta[3]) + nz * oz * sin(theta[2] + theta[3]) * cos(theta[3]) * sin(theta[1]) * sin(theta[2]) - nx * nz * oz * cos(theta[2]) * cos(theta[3]) * pow(sin(theta[1]), 2));

        dtheta[4] = 0;

        dtheta[5] = 0;

        dtheta[6] = 0;

        // 速度过快
        for (int i = 1; i <= N1; i++)
        {
            // 速度过快
            if (abs(dtheta[i]) > 1.001)
            {
                std::cout << std::endl
                          << "dtheta[" << i << "]:" << dtheta[i] << "!!!!!!!!!" << std::endl;
                dtheta[1] = 0;
                dtheta[2] = 0; // 肩关节避免奇异，使得二号电机转一个小角度
                dtheta[3] = 0;
                dtheta[4] = 0;
                dtheta[5] = 0;
                dtheta[6] = 0;
            }
        }
    }
    /*********↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓关节控制、末端控制↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*********/
    void modecontrol(int mode)
    {
        if (mode == JOINT_MODE)
        {
            if (flag_need_set_ModeV && !flag_pos_now) // 已经松开“↓”和→ 且需要开启速度模式  flag_need_set_ModeV只会在用过位置模式以后会变成1
            {
                std::cout << "速度模式已开启" << std::endl;
                firstNeedMode = need_v;
                flag_need_set_ModeV = 0;
            }

            /*************控制各电机速度********/
            /**
             * flag
             * 0 大臂旋转
             * 1 大臂前后
             * 2 小臂上下
             * 3 爪的上下
             * 4 爪的左右
             * 5 爪的上下
             * 6 爪的张合
             **/

            for (int i = 0; i < N1; i++)
                firstSpeed[i] = (1 + Is_ChangeV[i] * (cobra_Vel_level - 1)) * cobra_Vel[i] * flag[i] * cobra_motor_sign[i];

            for (int i = 0; i < N1; i++) // 将各个电机设置成速度模式
            {
                pController->activateActuatorMode(actuator[i], Actuator::Mode_Profile_Vel);
                cobra_SetV(actuator[i], firstSpeed[i], pos[i], LimitMax[i], LimitMin[i]);
            }
        }
        else if (mode == END_MODE)
        {
            if (handOrBaseMode) // 显示出末端坐标系手掌or基座
                cout << "\33[32m基座\033[0m坐标系控制：" << endl;
            else
                cout << "\33[32m手掌\033[0m坐标系控制：" << endl;
            cout << "dx:" << dx << ";dy:" << dy << ";dz:" << dz << ";dpitch:" << dpitch << ";dyaw:" << dyaw << ";droll:" << droll << ";" << endl;
            if (flag_d_theta_clear) // 把各电机速度清零
                for (int i = 1; i <= N1; i++)
                {
                    dtheta[i] = 0;
                }

            CartesianToJoint(handOrBaseMode); // 解算速度

            if (flag_need_set_ModeV && !flag_pos_now) // 已经松开“↓”和→ 且需要开启速度模式  flag_need_set_ModeV只会在用过位置模式以后会变成1
            {
                std::cout << "速度模式已开启" << std::endl;
                firstNeedMode = need_v;
                flag_need_set_ModeV = 0;
            }
            for (int i = 1; i <= N1; i++)
            {
                // 速度过快
                if (abs(dtheta[i]) > 1.001)
                {
                    std::cout << std::endl
                              << "dtheta[" << i << "]:" << dtheta[i] << "!!!!!!!!!" << std::endl;
                    dtheta[1] = 0;
                    dtheta[2] = 0; // 肩关节避免奇异，使得二号电机转一个小角度
                    dtheta[3] = 0;
                    dtheta[4] = 0;
                    dtheta[5] = 0;
                    dtheta[6] = 0;
                }
            }
            for (int i = 0; i < N1; i++)
                firstSpeed[i] = DH_sign[i] * dtheta[i + 1] * Motor_Reduction_Ratio[i] * 60.0 / (3.1415926 * 2);
            /*********↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓上机测试可注释这段，靠关节控制给速度并动作，末端控制看结果而不动作↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*********/
            for (int i = 0; i < N1; i++) // 将各个电机设置成速度模式
            {
                pController->activateActuatorMode(actuator[i], Actuator::Mode_Profile_Vel);
                cobra_SetV(actuator[i], firstSpeed[i], pos[i], LimitMax[i], LimitMin[i]);
            }
            /*********↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑上机测试可注释这段，靠关节控制给速度并动作，末端控制看结果而不动作↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑*********/
        }
    }

    /***************handle joy input***************/
    void joyCallback(const sensor_msgs::Joy &joy)
    {
        /*********↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓机械臂PS4手柄控制↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*********/
        if (now_control == cobra)
        {
            if (joy.axes[6] != 1 && joy.axes[6] != -1 && joy.axes[7] == -1) // 左键右键没有按下,且按着↓键  一般关节遥控
            {
                endOrJointMode = JOINT_MODE;
                // 大臂旋转
                flag[0] = joy.axes[0]; // 大臂旋转
                if (abs(flag[0]) < 0.2)
                    flag[0] = 0;

                // 大臂上下
                if (abs(joy.axes[4]) > abs(joy.axes[3])) // 防止摇杆按歪
                {
                    flag[1] = joy.axes[4]; // 大臂上下
                    flag[4] = 0;
                    if (abs(flag[1]) < 0.2)
                        flag[1] = 0;
                }

                // 爪的左右
                else
                {
                    flag[4] = joy.axes[3]; // 爪的左右
                    flag[1] = 0;
                    if (abs(flag[4]) < 0.2)
                        flag[4] = 0;
                }

                // 爪的张合
                if (joy.axes[2] != 0)
                    flag[6] = joy.buttons[4] + (joy.axes[2] - 1) / 2;

                // 爪的旋转
                flag[5] = joy.buttons[1] - joy.buttons[3];

                // 爪的上下
                if (joy.axes[5] != 0)
                    flag[3] = joy.buttons[5] + (joy.axes[5] - 1) / 2;

                // 小臂的上下
                flag[2] = joy.buttons[2] - joy.buttons[0];

                // 加减速
                if (joy.buttons[9] == 1 && joy.buttons[10] != 1) // 加速 档位
                {
                    cobra_Vel_level++;
                    if (cobra_Vel_level >= cobra_Vel_Total_level)
                        cobra_Vel_level = cobra_Vel_Total_level;
                }

                if (joy.buttons[8] == 1 && joy.buttons[10] != 1) // 减速 档位
                {
                    cobra_Vel_level--;
                    if (cobra_Vel_level <= 1)
                        cobra_Vel_level = 1;
                }
            }

            if (joy.axes[7] != -1) // 松掉↓键，清除flag
            {
                for (int i = 0; i < 6; i++)
                    flag[i] = 0;
            }
            if (joy.axes[6] == -1 && joy.axes[7] == -1) // ↓→键按着,一键模式
            {
                endOrJointMode = JOINT_MODE;
                if (joy.axes[4] >= 0.9) // 一键展开
                {
                    flag_extend = 1;
                    flag_need_set_ModeP = 1; // 现在需要设置成位置模式
                    flag_pos_now = 1;
                }
                if (joy.axes[4] <= -0.9) // 一键还原
                {
                    flag_back = 1;
                    flag_need_set_ModeP = 1; // 现在需要设置成位置模式
                    flag_pos_now = 1;
                }
                if (joy.axes[5] == -1 && joy.buttons[1] != 1 && joy.buttons[3] != 1) // R2
                {
                    flagA = 1;
                    flag_need_set_ModeP = 1; // 现在需要设置成位置模式
                    flag_pos_now = 1;
                }
                if (joy.buttons[1] == 1 && joy.buttons[0] != 1 && joy.buttons[2] != 1) // 单独按B
                {
                    flagB = 1;
                    flag_need_set_ModeP = 1; // 现在需要设置成位置模式
                    flag_pos_now = 1;
                }
                if (joy.buttons[3] == 1 && joy.buttons[0] != 1 && joy.buttons[2] != 1) // 单独按X
                {
                    flagX = 1;
                    flag_need_set_ModeP = 1; // 现在需要设置成位置模式
                    flag_pos_now = 1;
                }
                if (joy.buttons[5] == 1 && joy.buttons[1] != 1 && joy.buttons[3] != 1) // 单独R1
                {
                    flagY = 1;
                    flag_need_set_ModeP = 1; // 现在需要设置成位置模式
                    flag_pos_now = 1;
                }

                if (joy.buttons[2] == 1 && joy.buttons[1] == 1) // 按下Y和double B
                {
                    flagBY = 1;
                    flagB = 0;
                    flagY = 0;
                    flag_need_set_ModeP = 1; // 现在需要设置成位置模式
                    flag_pos_now = 1;
                }
                if (joy.buttons[2] == 1 && joy.buttons[3] == 1) // 按下Y和X
                {
                    flagXY = 1;
                    flagX = 0;
                    flagY = 0;
                    flag_need_set_ModeP = 1; // 现在需要设置成位置模式
                    flag_pos_now = 1;
                }
                if (joy.buttons[0] == 1 && joy.buttons[3] == 1) // 按下A和X
                {
                    flagAX = 1;
                    flagX = 0;
                    flagA = 0;
                    flag_need_set_ModeP = 1; // 现在需要设置成位置模式
                    flag_pos_now = 1;
                }
                if (joy.buttons[0] == 1 && joy.buttons[1] == 1) // 按下A和B
                {
                    flagAB = 1;
                    flagA = 0;
                    flagB = 0;
                    flag_need_set_ModeP = 1; // 现在需要设置成位置模式
                    flag_pos_now = 1;
                }
            }

            if (joy.axes[6] != 1 && joy.axes[6] != -1 && joy.buttons[10] == 1) // 没有按左右 按下开灯键
            {
                ros::NodeHandle n;
                if (joy.buttons[2] == 1) // 剪刀末端
                {
                    n.setParam("firstCobra_d6", 0.512);
                    n.setParam("firstCobra_a6", 0.00);
                }
                if (joy.buttons[0] == 1) // 爪子末端
                {
                    n.setParam("firstCobra_d6", 0.284);
                    n.setParam("firstCobra_a6", 0.00);
                }
                if (joy.buttons[5] == 1) // 开门末端
                {
                    n.setParam("firstCobra_d6", 0.284);
                    n.setParam("firstCobra_a6", 0.15);
                }
            }

            if (joy.axes[6] != -1 || joy.axes[7] != -1) // 松掉下或右
            {
                firstNeedMode = need_v; // 设置速度模式
            }

            if (joy.axes[7] != -1) // 末端控制
            {
                // 释放flag值
                flag_extend = 0;
                flag_pos_now = 0; // 位置控制完毕
                flag_back = 0;
                for (int i = 0; i < 6; i++)
                    flag[i] = 0;
                // endOrJointMode = 1;//基础都为普通速度控制
                // flag_d_theta_clear = 1;
                flagX = 0;
                flagY = 0;
                flagA = 0;
                flagB = 0;
                flagAB = 0;
                flagAX = 0;
                flagXY = 0;
                flagBY = 0;
                endOrJointMode = END_MODE;

                // ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓保护机制
                if (abs(joy.axes[0]) < 0.2 && abs(joy.axes[1]) < 0.2 && abs(joy.axes[3]) < 0.2 && abs(joy.axes[4]) < 0.2 && joy.axes[5] > 0.9 && joy.buttons[1] == 0 && joy.buttons[3] == 0 && joy.buttons[5] == 0)
                {
                    flag_d_theta_clear = 1;
                }
                else
                    flag_d_theta_clear = 0;
                // ↑↑↑↑↑↑↑保护机制↑↑↑↑↑↑

                if (joy.buttons[12] == 1) // 切换末端控制模式 基座坐标系or末端坐标系
                    handOrBaseMode = !handOrBaseMode;
                if (joy.buttons[9] == 1) // 末端加速
                {
                    end_mode_speed_level++;
                    if (end_mode_speed_level >= end_mode_Total_level)
                        end_mode_speed_level = end_mode_Total_level;
                }
                if (joy.buttons[8] == 1) // 末端减速
                {
                    end_mode_speed_level--;
                    if (end_mode_speed_level <= 1)
                        end_mode_speed_level = 1;
                }
                if (handOrBaseMode) // 基座坐标系   3轴肩关节下,末端相对基座运动仅有3个自由度dpitch dyaw droll，但dx,dy,dz也可以存在，会受到约束，持续施加dxdydz最终使得臂伸直展开
                {
                    if (abs(joy.axes[0]) > 0.2)
                        flag2[4] = joy.axes[0];
                    else
                        flag2[4] = 0;
                    if (abs(joy.axes[1]) > 0.2)
                        flag2[3] = joy.axes[1];
                    else
                        flag2[3] = 0;
                    dpitch = rotate_resolution * end_mode_speed_level * flag2[4];

                    dyaw = rotate_resolution * end_mode_speed_level * flag2[3];

                    if (joy.axes[5] != 0)
                        dx = speed_resolution * end_mode_speed_level * (joy.buttons[5] + (joy.axes[5] - 1) / 2); // 向上为正  上下

                    if (joy.axes[7] != 1)
                        droll = rotate_resolution * end_mode_speed_level * (joy.buttons[1] - joy.buttons[3]);

                    flag2[1] = joy.axes[3];
                    if (abs(flag2[1]) < 0.2)
                        flag2[1] = 0;
                    dy = -speed_resolution * end_mode_speed_level * flag2[1]; // 左右

                    flag2[2] = joy.axes[4];
                    if (abs(flag2[2]) < 0.2)
                        flag2[2] = 0;
                    dz = speed_resolution * end_mode_speed_level * flag2[2]; // 前后
                }

                if (!handOrBaseMode) // 末端坐标系  3轴肩关节下，末端相对末端运动只有3个自由度，dx dy droll
                {
                    if (abs(joy.axes[0]) > 0.2)
                        flag2[4] = joy.axes[0];
                    else
                        flag2[4] = 0;
                    if (abs(joy.axes[1]) > 0.2)
                        flag2[3] = joy.axes[1];
                    else
                        flag2[3] = 0;
                    dpitch = rotate_resolution * end_mode_speed_level * flag2[4];

                    dyaw = rotate_resolution * end_mode_speed_level * flag2[3];

                    if (joy.axes[5] != 0)
                        dx = speed_resolution * end_mode_speed_level * (joy.buttons[5] + (joy.axes[5] - 1) / 2); // 向上为正  上下

                    if (joy.axes[7] != 1)
                        droll = rotate_resolution * end_mode_speed_level * (joy.buttons[1] - joy.buttons[3]);

                    flag2[1] = joy.axes[3];
                    if (abs(flag2[1]) < 0.2)
                        flag2[1] = 0;
                    dy = -speed_resolution * end_mode_speed_level * flag2[1]; // 左右

                    flag2[2] = joy.axes[4];
                    if (abs(flag2[2]) < 0.2)
                        flag2[2] = 0;
                    dz = speed_resolution * end_mode_speed_level * flag2[2]; // 前后
                }
            }
        }
        else // 松掉 ↓ 后 释放标志位
        {
            // std::cout << "按键释放" << std::endl;
            for (int i = 0; i <= N1; i++)
            {
                flag[i] = 0;
                dtheta[i] = 0;
            }
            for (int i = 0; i <= 4; i++)
            {
                flag2[i] = 0;
            }
            flag_d_theta_clear = 1;
        }
    }
    /***************时间中断，控制部分***************/
    void timeController(const ros::TimerEvent &e)
    {
        All_Motor_IsOnlineAndEnable = true;
        for (int i = 0; i < N; i++) // 有几个电机串联在CAN总线上  i就小于几  待改地方！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！！虽然只使用其中6个进行逆解，但此处7R，有7个电机，所以为7
        {
            All_Motor_IsOnlineAndEnable = All_Motor_IsOnlineAndEnable & cobra_isOnlineAndEnable(actuator[i]);
            if (!All_Motor_IsOnlineAndEnable)
                break;
            else
                enableOK = true;
        }
        /*如果有设备掉线或者失能*/ // ↓↓↓↓↓↓↓↓↓↓↓↓↓初始化↓↓↓↓↓↓↓↓↓↓↓↓↓
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
                    NOWMODE = VMODE;
                }
            }
            std::cout << "Error!!!!!!!!网线没好!!!!!!" << std::endl;
            enableOK = false;
        }
        else
        {
            /**********取得底层电机位置********/
            for (int i = 0; i < N; i++)
            {
                original_pos[i] = pController->getPosition(actuator[i], true); // 得到底层电机位置 单位：转

                theta[i + 1] = 3.1415926 * (original_pos[i] - pos2theta[i]) * DH_sign[i] / 180; // 得到姿态角度，单位：弧度  用于计算的角度不能用round

                pos[i] = round((360.0 / Motor_Reduction_Ratio[i]) * original_pos[i]); // 除以减速比乘以360转化成目视角度

                temperature[i] = pController->getMotorTemperature(actuator[i], true); // 獲取電機溫度

                DianLiu[i] = pController->getCurrent(actuator[i], true);
            }
        }
        if (!enableOK)
            cout << "\033[47;31m电机未初始化好!!!请等待！！！\033[0m" << endl
                 << "\033[47;31m很久没好则为网络（网线）未连接好or机械臂未上电！！！\033[0m" << endl;
        else
        {
            ros::NodeHandle n;
            n.param<int>("cobraNum1", N1, 7);
            n.param<int>("cobraNum2", N2, 0);
            // 机械臂参数
            n.param<double>("kSpeedResolution", speed_resolution, 0.05);
            n.param<double>("kRotateResolution", rotate_resolution, 0.2);
            n.param<double>("d3", d3, 0.4);
            N = N1 + N2;

            cout << "d3:  " << d3 << endl;

            cout << "SpeedLevel:" << cobra_Vel_level << endl;
            std::cout << "各电机角度，单位：度以及底层电机控制量速度，单位：rpm:" << std::endl;
            for (int i = 0; i < N; i++)
            {
                std::cout << "电机" << i + 1 << "[ID:" << int(actuator[i]) << "] Angle:" << fixed << setprecision(2) << setiosflags(ios::left) << setw(7) << pos[i] << "[" << setw(4) << LimitMin[i] << "~" << setw(3) << LimitMax[i] << "];";
                std::cout << "速度控制指令: " << setiosflags(ios::left) << setw(6) << firstSpeed[i] << ";";
                if (endOrJointMode == JOINT_MODE)
                {
                    std::cout << "flag[" << i << "]:" << flag[i] << ";" << std::endl;
                }
                else if (endOrJointMode == END_MODE)
                {
                    std::cout << "dtheta" << i << "：" << dtheta[i] << ";" << std::endl;
                }
            }
            std::cout << std::endl;

            cout << "Current:";
            for (int i = 0; i < N; i++)
                cout << "[" << round(100 * DianLiu[i]) / 100 << "];";
            cout << endl;

            cout << "Temperature:";
            for (int i = 0; i < N; i++)
                cout << temperature[i] << ";";
            cout << endl;

            modecontrol(endOrJointMode); // 在电机使能的情况下进行控制

            if (endOrJointMode == JOINT_MODE)
                cout << "\33[32m关节控制\033[0m" << endl;
            if (endOrJointMode == END_MODE)
                cout << "\33[32m末端控制\033[0m" << endl;
        }
        for (int i = 0; i <= 20; i++) // 清屏
        {
            printf(cursup);
            printf(cursclean);
        }
    }

private:
    ros::Subscriber joy_sub_;

    ros::Timer controller;
    bool now_control = true;
    bool cobra = true;
    int NOWMODE = 0;
    float VMODE = 1;
    float PMODE = 2;
    int N1 = 7, N2 = 0, N = N1 + N2; // CAN上电机个数
    bool enableOK = false;
    int i;
    int need_v = 1;                 // 标志位，程序中不应改变
    int need_p = 2;                 // 标志位，
    double rotate_resolution = 0.2; // 末端旋转速度的分辨率（越大速度越快）
    double speed_resolution = 0.05; // 末端平移速度的分辨率（越大速度越快）

    float firstNeedMode = 0;
    float firstNowMode = 0;
    int flagX = 0, flagY = 0, flagA = 0, flagB = 0, flagAB = 0, flagAX = 0, flagXY = 0, flagBY = 0; // 八种位置模式组合键

    float flag_button = 0;
    int cobra_Vel_level = 1; // 机械臂单机速度档位
    int end_mode_speed_level = 2;

    int flag_extend = 0;         // 一键展开
    int flag_need_set_ModeV = 0; // 需要设置成速度模式
    int flag_need_set_ModeP = 0; // 需要设置成位置模式
    int flag_back = 0;           // 一键归位

    int flag_end_stop = 0;           // 末端控制停止标志符
    int endOrJointMode = JOINT_MODE; // 控制模式，1为一般控制，2为末端控制
    int flag_pos_now = 0;            // 现在正在位置控制  注意：凡是用到位置模式都需要将该标志位改为1；
    bool All_Motor_IsOnlineAndEnable = true;
    int flag_d_theta_clear = 1;
    bool handOrBaseMode = false;

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
    double d3 = 0.4;
    double nx, ox, ax, px, ny, oy, ay, py, nz, oz, az, pz;
    double dx = 0, dy = 0, dz = 0, dpitch = 0, dyaw = 0, droll = 0, dax = 0, day = 0, daz = 0;
    double dnx = 0, dny = 0, dnz = 0, dpx = 0, dpy = 0, dpz = 0, dox = 0, doy = 0, doz = 0;
    double dtheta[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 末端控制状态控制速度 ，下标从1开始，theta[0]无用。单位弧度。
    double theta[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // 末端控制状态角度 ，下标从1开始，theta[0]无用。单位弧度。

    float flag[7] = {0, 0, 0, 0, 0, 0, 0};                                // 普通遥控各电机遥控指令
    double flag2[6] = {0, 0, 0, 0, 0, 0};                                 // 末端遥控各电机指令  1:d_x   2:d_z  3:d_pitch   4:d_yaw
    double pos[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};       // 各电机位置信息 单位：°
    double pos2theta[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 当前位置到末端控制状态角转换差值
    int DH_sign[7] = {-1, 1, -1, -1, -1, 1, 1};                           // 电机摆放对DH建模正方向的影响
    int cobra_motor_sign[7] = {-1, -1, -1, -1, -1, -1, 1};                // 电机与遥控器正反转适配

    // 机械臂速度： 6 * cobra_Vel/Motor_Reduction_Ratio 度/秒
    double cobra_Vel[7] = {167, 167, 167, 167, 167, 167, 167};    // 各底层电机基础速度   ,单位：每分钟多少转  具体到机械臂角度计算： cobra_Vel/60 * 360/Motor_Reduction_Ratio = 6 * cobra_Vel/Motor_Reduction_Ratio 度/秒
    int Motor_Reduction_Ratio[7] = {36, 36, 36, 36, 36, 36, 36};  // 各电机减速比
    int LimitMax[7] = {180, 160, 160, 160, 160, 160, 160};        // 各电机最大限位 单位：° 度
    int LimitMin[7] = {-180, -160, -160, -160, -160, -160, -160}; // 各电机最小限位 单位：° 度

    double original_pos[7] = {0, 0, 0, 0, 0, 0, 0};
    double firstSpeed[7] = {0, 0, 0, 0, 0, 0, 0};

    int cobra_Vel_Total_level = 3; // 机械臂速度档位数， 成倍增加，即3档是1档的3倍速，2档是1档的2倍速
    int end_mode_Total_level = 3;

    int Is_ChangeV[7] = {1, 1, 1, 1, 1, 1, 1}; // 各电机是否受加减速影响，1为可以加减速，0为屏蔽加减速。

    float temperature[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 电机的温度
    float DianLiu[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};     // 电机的dianliu
    int endV = 50;                                                         // 末端三个关节的一键模式的速度
    // ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑参数调节↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
};

int main(int argc, char *argv[])
{
    /*******************************ROS*****************************/
    ros::init(argc, argv, "test_tgcontrol");
    nubot_arm_node arm;
    ros::NodeHandle n;
    //    n.param<int>("cobraNum1",N1,6);
    //    n.param<int>("cobraNum2",N2,6);

    ros::spin();
    return 0;
}
