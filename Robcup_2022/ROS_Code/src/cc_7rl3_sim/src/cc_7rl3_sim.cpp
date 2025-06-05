/**
 * cc_7rl3_sim
 * 关节限位   (− 180 ~ 180)  (− 42 ~ 85)  (− 20 ~ 120)  (− 300 ~ 300)  (− 120 ~ 120)  ( − 360 ~ 360)
 * 机械臂仿真
 * 20230310
 * 吴拓昌
 * 机械臂参数偏置部分需要些许调整
 **/

#include <ros/ros.h>

#include <time.h>

#include <signal.h>
#include <string.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <iostream>
#include <thread>
#include <eigen3/Eigen/Dense>
#include "nubot_msgs/angle_position.h"
// #include "nubot_msgs/cobraToRviz.h"
// #include "nubot_msgs/link_msg.h"

// #include "nubot_msgs/innfosPub.h"
// #include "nubot_msgs/innfosSub.h"

#include "nubot_msgs/simPub.h"
#include "nubot_msgs/simSub.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"
#define JOINT_MODE 1
#define END_MODE 2

using namespace std;
using namespace Eigen;

int Sgn(double d)
{
    if (d < 0)
        return -1;
    else if (d == 0)
        return 0;
    else
        return 1;
};
class nubot_arm_node2
{
public:
    int time_fl = 0; // 计时flag
    nubot_arm_node2()
    {
        ros::NodeHandle n;
        /*********节点*********/
        joy_sub_ = n.subscribe("/joy", 1, &nubot_arm_node2::joyCallback, this);
        controller = n.createTimer(ros::Duration(1.0 / 100.0),
                                   &nubot_arm_node2::timeController, this, false);
        cobra_angle_pub_ = n.advertise<nubot_msgs::angle_position>("/timon_angle", 1);
        // control_sub_ = n.subscribe("control_flag", 1, &nubot_arm_node2::linkCallback, this);
        // cobra_to_rviz_pub_ = n.advertise<nubot_msgs::cobraToRviz>("cobraToRviz", 1);
        firstCobraPubTosim_ = n.advertise<nubot_msgs::simSub>("/simSub_", 1, true);

        firstCobraSubFromsim_ = n.subscribe("/simPub_", 1, &nubot_arm_node2::firstCobraCallback, this);
        firstCobraPubTosimMsg.firstNeedMode = need_v;
    }

    ~nubot_arm_node2() // 关闭节点
    {
        std::cout << "Node closing！ Begin initializing" << std::endl;
    }

    void firstCobraCallback(const nubot_msgs::simPub &firstCobra)
    {
        // for (i = 0; i < N1; i++)
        // {
        //     pos[i] = firstCobra.firstOriginalPos[i];
        //     pos[i] = round((360.0 / Motor_Reduction_Ratio[i]) * pos[i]);           //转化成角度
        //     theta[i + 1] = 3.1415926 * (pos[i] - pos2theta[i]) * DH_sign[i] / 180; //得到姿态角度
        // }
        for (i = 0; i < N1; i++) // 仿真使用
        {
            pos[i] = firstCobra.firstOriginalPos[i];
            theta[i + 1] = 3.1415926 * (pos[i] - pos2theta[i]) * DH_sign[i] / 180; // 角度转换为弧度
            if (flag_init)                                                         // online后第一次读取角度数据给它存起来，且开一次节点仅存一次，方便臂的归位
                pos_init[i] = pos[i];
        }
        flag_init = false;
        enableOK = firstCobra.enableOK;
        // time_fl = 20; //计时器
    }

    void CartesianToJoint(bool mode)
    {
        // 求出姿态
        nx = sin(theta[6]) * (cos(theta[4]) * sin(theta[1]) + sin(theta[4]) * (cos(theta[1]) * cos(theta[2]) * sin(theta[3]) + cos(theta[1]) * cos(theta[3]) * sin(theta[2]))) + cos(theta[6]) * (cos(theta[5]) * (sin(theta[1]) * sin(theta[4]) - cos(theta[4]) * (cos(theta[1]) * cos(theta[2]) * sin(theta[3]) + cos(theta[1]) * cos(theta[3]) * sin(theta[2]))) + sin(theta[5]) * (cos(theta[1]) * sin(theta[2]) * sin(theta[3]) - cos(theta[1]) * cos(theta[2]) * cos(theta[3])));

        ny = -sin(theta[6]) * (cos(theta[1]) * cos(theta[4]) - sin(theta[4]) * (cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + cos(theta[3]) * sin(theta[1]) * sin(theta[2]))) - cos(theta[6]) * (cos(theta[5]) * (cos(theta[1]) * sin(theta[4]) + cos(theta[4]) * (cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + cos(theta[3]) * sin(theta[1]) * sin(theta[2]))) - sin(theta[5]) * (sin(theta[1]) * sin(theta[2]) * sin(theta[3]) - cos(theta[2]) * cos(theta[3]) * sin(theta[1])));

        nz = -cos(theta[6]) * (sin(theta[2] + theta[3]) * sin(theta[5]) - cos(theta[2] + theta[3]) * cos(theta[4]) * cos(theta[5])) - cos(theta[2] + theta[3]) * sin(theta[4]) * sin(theta[6]);

        ox = cos(theta[6]) * (cos(theta[4]) * sin(theta[1]) + sin(theta[4]) * (cos(theta[1]) * cos(theta[2]) * sin(theta[3]) + cos(theta[1]) * cos(theta[3]) * sin(theta[2]))) - sin(theta[6]) * (cos(theta[5]) * (sin(theta[1]) * sin(theta[4]) - cos(theta[4]) * (cos(theta[1]) * cos(theta[2]) * sin(theta[3]) + cos(theta[1]) * cos(theta[3]) * sin(theta[2]))) + sin(theta[5]) * (cos(theta[1]) * sin(theta[2]) * sin(theta[3]) - cos(theta[1]) * cos(theta[2]) * cos(theta[3])));

        oy = sin(theta[6]) * (cos(theta[5]) * (cos(theta[1]) * sin(theta[4]) + cos(theta[4]) * (cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + cos(theta[3]) * sin(theta[1]) * sin(theta[2]))) - sin(theta[5]) * (sin(theta[1]) * sin(theta[2]) * sin(theta[3]) - cos(theta[2]) * cos(theta[3]) * sin(theta[1]))) - cos(theta[6]) * (cos(theta[1]) * cos(theta[4]) - sin(theta[4]) * (cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + cos(theta[3]) * sin(theta[1]) * sin(theta[2])));

        oz = sin(theta[6]) * (sin(theta[2] + theta[3]) * sin(theta[5]) - cos(theta[2] + theta[3]) * cos(theta[4]) * cos(theta[5])) - cos(theta[2] + theta[3]) * cos(theta[6]) * sin(theta[4]);

        ax = sin(theta[5]) * (sin(theta[1]) * sin(theta[4]) - cos(theta[4]) * (cos(theta[1]) * cos(theta[2]) * sin(theta[3]) + cos(theta[1]) * cos(theta[3]) * sin(theta[2]))) - cos(theta[5]) * (cos(theta[1]) * sin(theta[2]) * sin(theta[3]) - cos(theta[1]) * cos(theta[2]) * cos(theta[3]));

        ay = -sin(theta[5]) * (cos(theta[1]) * sin(theta[4]) + cos(theta[4]) * (cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + cos(theta[3]) * sin(theta[1]) * sin(theta[2]))) - cos(theta[5]) * (sin(theta[1]) * sin(theta[2]) * sin(theta[3]) - cos(theta[2]) * cos(theta[3]) * sin(theta[1]));

        az = sin(theta[2] + theta[3]) * cos(theta[5]) + cos(theta[2] + theta[3]) * cos(theta[4]) * sin(theta[5]);

        px = d4 * cos(theta[2] + theta[3]) * cos(theta[1]) - a3 * cos(theta[1]) * sin(theta[2]) + d6 * cos(theta[2] + theta[3]) * cos(theta[1]) * cos(theta[5]) + d6 * sin(theta[1]) * sin(theta[4]) * sin(theta[5]) - d6 * cos(theta[1]) * cos(theta[2]) * cos(theta[4]) * sin(theta[3]) * sin(theta[5]) - d6 * cos(theta[1]) * cos(theta[3]) * cos(theta[4]) * sin(theta[2]) * sin(theta[5]);

        py = d4 * cos(theta[2] + theta[3]) * sin(theta[1]) - a3 * sin(theta[1]) * sin(theta[2]) + d6 * cos(theta[2] + theta[3]) * cos(theta[5]) * sin(theta[1]) - d6 * cos(theta[1]) * sin(theta[4]) * sin(theta[5]) - d6 * cos(theta[2]) * cos(theta[4]) * sin(theta[1]) * sin(theta[3]) * sin(theta[5]) - d6 * cos(theta[3]) * cos(theta[4]) * sin(theta[1]) * sin(theta[2]) * sin(theta[5]);

        pz = d1 + d4 * sin(theta[2] + theta[3]) + a3 * cos(theta[2]) + (d6 * cos(theta[2] + theta[3]) * sin(theta[4] + theta[5])) / 2 + d6 * sin(theta[2] + theta[3]) * cos(theta[5]) - (d6 * sin(theta[4] - theta[5]) * cos(theta[2] + theta[3])) / 2;

        if (mode) // 基座坐标系
        {
            // std::cout << "基座";
            dnx = dyaw * nz - droll * ny;
            dny = droll * nx - dpitch * nz;
            dnz = dpitch * ny - dyaw * nx;

            dox = dyaw * oz - droll * oy;
            doy = droll * ox - dpitch * oz;
            doz = dpitch * oy - dyaw * ox;

            dax = az * dyaw - ay * droll;
            day = ax * droll - az * dpitch;
            daz = ay * dpitch - ax * dyaw;

            dpx = dx - droll * py + dyaw * pz;
            dpy = dy - dpitch * pz + droll * px;
            dpz = dz + dpitch * py - dyaw * px;
        }
        if (!mode) // 末端坐标系  绕末端z轴rz=roll      绕末端y轴 ry yaw     绕末端x轴 rx pitch
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
        // 解算速度
        dtheta[1] = (dpy * cos(theta[1]) - dpx * sin(theta[1]) + d6 * dax * sin(theta[1]) - d6 * day * cos(theta[1])) / (px * cos(theta[1]) - d6 * (ax * cos(theta[1]) + ay * sin(theta[1])) + py * sin(theta[1]));

        dtheta[2] = -(dpy * cos(theta[2] + theta[3]) + dpz * sin(theta[2] + theta[3]) * sin(theta[1]) - d6 * day * cos(theta[2] + theta[3]) - d4 * dtheta[1] * pow(cos(theta[2] + theta[3]), 2) * cos(theta[1]) - d6 * daz * sin(theta[2] + theta[3]) * sin(theta[1]) + a3 * dtheta[1] * cos(theta[2] + theta[3]) * cos(theta[1]) * sin(theta[2])) / (sin(theta[1]) * (a3 * cos(theta[2] + theta[3]) * cos(theta[2]) + a3 * sin(theta[2] + theta[3]) * sin(theta[2])));

        dtheta[3] = -(d6 * daz - dpz + dtheta[2] * (d4 * cos(theta[2] + theta[3]) - a3 * sin(theta[2]))) / (d4 * cos(theta[2] + theta[3]));

        dtheta[5] = (dax * (cos(theta[1]) * sin(theta[2]) * sin(theta[3]) - cos(theta[1]) * cos(theta[2]) * cos(theta[3])) + day * (sin(theta[1]) * sin(theta[2]) * sin(theta[3]) - cos(theta[2]) * cos(theta[3]) * sin(theta[1])) - daz * (cos(theta[2]) * sin(theta[3]) + cos(theta[3]) * sin(theta[2])) - dtheta[1] * (ay * cos(theta[1]) * cos(theta[2]) * cos(theta[3]) - ax * cos(theta[2]) * cos(theta[3]) * sin(theta[1]) - ay * cos(theta[1]) * sin(theta[2]) * sin(theta[3]) + ax * sin(theta[1]) * sin(theta[2]) * sin(theta[3])) + dtheta[2] * (az * sin(theta[2]) * sin(theta[3]) - az * cos(theta[2]) * cos(theta[3]) + ax * cos(theta[1]) * cos(theta[2]) * sin(theta[3]) + ax * cos(theta[1]) * cos(theta[3]) * sin(theta[2]) + ay * cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + ay * cos(theta[3]) * sin(theta[1]) * sin(theta[2])) + dtheta[3] * (az * sin(theta[2]) * sin(theta[3]) - az * cos(theta[2]) * cos(theta[3]) + ax * cos(theta[1]) * cos(theta[2]) * sin(theta[3]) + ax * cos(theta[1]) * cos(theta[3]) * sin(theta[2]) + ay * cos(theta[2]) * sin(theta[1]) * sin(theta[3]) + ay * cos(theta[3]) * sin(theta[1]) * sin(theta[2]))) / sin(theta[5]);

        dtheta[4] = (dtheta[1] * (ax * cos(theta[1]) + ay * sin(theta[1])) - day * cos(theta[1]) + dax * sin(theta[1]) - dtheta[5] * cos(theta[5]) * sin(theta[4])) / (cos(theta[4]) * sin(theta[5]));

        dtheta[6] = -(dtheta[4] * (cos(theta[6]) * sin(theta[4]) + cos(theta[4]) * cos(theta[5]) * sin(theta[6])) + dtheta[1] * (ox * cos(theta[1]) + oy * sin(theta[1])) - doy * cos(theta[1]) + dox * sin(theta[1]) - dtheta[5] * sin(theta[4]) * sin(theta[5]) * sin(theta[6])) / (cos(theta[4]) * sin(theta[6]) + cos(theta[5]) * cos(theta[6]) * sin(theta[4]));

        for (int i = 1; i <= 6; i++)
        {
            // 速度过快
            if (abs(dtheta[i]) > 1.601)
            {
                std::cout << std::endl
                          << "dtheta[" << i << "]:" << dtheta[i] << "!!!!!!!!!" << std::endl;
                dtheta[1] = 0;
                dtheta[2] = 0;
                dtheta[3] = 0;
                dtheta[4] = 0;
                dtheta[5] = 0;
                dtheta[6] = 0;
            }
        }
    }

    // void linkCallback(const nubot_msgs::link_msg &pumbaa_or_cobra)
    // {
    //     now_control = pumbaa_or_cobra.control_flag;
    //     // std::cout << now_control << std::endl;
    // }

    /***************handle joy input***************/
    void joyCallback(const sensor_msgs::Joy &joy)
    {
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
                if (joy.axes[4] >= 0.9)
                {
                    flag_extend = 1;
                    flag_need_set_ModeP = 1; // 现在需要设置成位置模式
                    flag_pos_now = 1;
                }
                if (joy.axes[4] <= -0.9)
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
                firstCobraPubTosimMsg.firstNeedMode = need_v; // 设置速度模式
                firstCobraPubTosim_.publish(firstCobraPubTosimMsg);
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
                if (joy.buttons[11] == 1) // 切换自主控制模式 遥控or自主
                {
                    auto_mode = !auto_mode;
                    flag_auto = 1;
                    if (auto_mode)
                    {
                        endOrJointMode = END_MODE;
                        handOrBaseMode = true; // 基座坐标系
                    }
                }
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
                if (handOrBaseMode) // 基座坐标系
                {
                    if (joy.axes[5] != 0)
                        dz = speed_resolution * end_mode_speed_level * (joy.buttons[5] + (joy.axes[5] - 1) / 2); // 向上为正  上下

                    flag2[1] = joy.axes[3];
                    if (abs(flag2[1]) < 0.2)
                        flag2[1] = 0;
                    dy = speed_resolution * end_mode_speed_level * flag2[1]; // 左右

                    flag2[2] = joy.axes[4];
                    if (abs(flag2[2]) < 0.2)
                        flag2[2] = 0;
                    dx = speed_resolution * end_mode_speed_level * flag2[2]; // 前后
                }

                if (!handOrBaseMode) // 末端坐标系
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
            for (int i = 0; i <= 6; i++)
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
    void auto_arm() // 自主模式下的机械臂运动控制，flag_auto==0则机械臂正在自主运动中，flag_auto==1为运动完成
    {
        float er = 0.5; // 电机运动到位的角度误差
        if (flag_auto == 1)
        {
            RHS.setZero();
            input_goal.setZero();

            RHS << nx, ox, ax, px, ny, oy, ay, py, nz, oz, az, pz, 0, 0, 0, 1;
            cout << "等待目标位置（末端坐标下）数据中" << endl;
            // ↓↓↓↓↓↓↓↓↓↓此处可迁移至rostopic消息的callback中↓↓↓↓↓↓↓↓↓↓↓
            const int SIZE = 3;
            double nums[SIZE];
            cout << "Enter " << SIZE << " float numbers range(-0.15~0.15): "; // 此模型较短，运动范围有限
            for (int i = 0; i < SIZE; i++)
            {
                cin >> nums[i];
            }
            input_goal << 1, 0, 0, nums[0], 0, 1, 0, nums[1], 0, 0, 1, nums[2], 0, 0, 0, 1;
            flag_auto = 0;
            // ↑↑↑↑↑↑↑↑↑↑此处可迁移至rostopic消息的callback中↑↑↑↑↑↑↑↑↑↑↑
            RHS = RHS * input_goal; // 只有flag_auto==1运动完成后才可以修改一次RHS，即目标位置的变换矩阵

            // ↓↓↓↓↓↓↓↓↓↓此处直接添加RHS逆解到关节角度的计算公式，注意判断角度多解的取舍，得到goal_theta↓↓↓↓↓↓↓↓↓↓↓

            // flag_need_set_ModeP = 1;
            //  ↑↑↑↑↑↑↑↑↑↑此处直接添加RHS逆解到关节角度的计算公式，注意判断角度多解的取舍，得到goal_theta↑↑↑↑↑↑↑↑↑↑↑
        }
        else
        {
            cout << "\33[32m自主运动中！！！\033[0m" << endl;
            if (abs(px - RHS(0, 3)) >= 0.01)
                dx = speed_resolution * end_mode_speed_level * 0.5 * -Sgn(px - RHS(0, 3));
            else
                dx = 0;
            if (abs(py - RHS(1, 3)) >= 0.01)
                dy = speed_resolution * end_mode_speed_level * 0.5 * -Sgn(py - RHS(1, 3));

            else
                dy = 0;
            if (abs(pz - RHS(2, 3)) >= 0.01)
                dz = speed_resolution * end_mode_speed_level * 0.5 * -Sgn(pz - RHS(2, 3));
            else
                dz = 0;
            // if ((pos[0] - goal_theta[0]) < er && (pos[1] - goal_theta[1]) < er &&(pos[2] - goal_theta[2]) < er&&(pos[3] - goal_theta[3]) < er&&(pos[4] - goal_theta[4]) < er&&(pos[5] - goal_theta[5]) < er)
            // {
            //         flag_auto = 1;
            //     if (flag_need_set_ModeV && !flag_need_set_ModeP) // 已经使用过速度模式且需要开启速度模式  flag_need_set_ModeV只会在用过位置模式以后会变成1
            //     {
            //         std::cout << "速度模式已开启" << std::endl;
            //         firstCobraPubTosimMsg.firstNeedMode = need_v;
            //         firstCobraPubTosim_.publish(firstCobraPubTosimMsg);
            //         flag_need_set_ModeV = 0;
            //     }
            // }

            if (dx == 0 && dy == 0 && dz == 0)
                flag_auto = 1;
        }
        // ↓↓↓↓↓↓↓↓↓↓若使用关节角度的控制，需要修改为电机位置控制模式，并发布相应位置控制消息↓↓↓↓↓↓↓↓↓↓↓
        CartesianToJoint(handOrBaseMode);
        std::cout << "dx:" << dx << ";dy:" << dy << ";dz:" << dz << ";dpitch:" << dpitch << ";dyaw:" << dyaw << ";droll:" << droll << ";" << std::endl;
        for (int i = 0; i < N1; i++)
            firstCobraPubTosimMsg.firstSpeed[i] = DH_sign[i] * dtheta[i + 1]; // 仿真环境下需要修改单位为弧度每秒

        // if (flag_need_set_ModeP) // 只执行一次这个子循环，即只设置一次位置模式，否则会出现：设置位置模式→控制电机位置→设置位置模式→控制电机位置→...一直循环这样会使电机不动。
        // {
        //     for (int i = 0; i < N1; i++) // 将各个电机设置成位置模式
        //     {
        //         firstCobraPubTosimMsg.firstNeedMode = need_p;
        //         firstCobraPubTosimMsg.firstPos[i] = goal_theta[i] / (360.0 / Motor_Reduction_Ratio[i]);
        //         firstCobraPubTosimMsg.firstPosMode = 1;
        //     }
        //     flag_need_set_ModeP = 0;
        //     flag_need_set_ModeV = 1;
        // }
        // firstCobraPubTosimMsg.flag_Auto = flag_auto;
        // ↑↑↑↑↑↑↑↑↑↑若使用关节角度的控制，需要修改为电机位置控制模式，并发布相应位置控制消息↑↑↑↑↑↑↑↑↑↑↑
        firstCobraPubTosim_.publish(firstCobraPubTosimMsg); // 可以在此处将flag_auto一同发布出去
    }
    /***************时间中断，控制部分***************/
    void timeController(const ros::TimerEvent &e)
    {
        if (!enableOK)
            cout << "\033[47;31m电机未初始化好!!!请等待！！！\033[0m" << endl
                 << "\033[47;31m很久没好则为网络（网线）未连接好or机械臂未上电！！！\033[0m" << endl;
        else
        {
            ros::NodeHandle n;
            n.param<int>("cobraNum1", N1, 6);
            n.param<int>("cobraNum2", N2, 0);
            // 机械臂参数
            n.param<double>("firstCobra_a3", a3, 0.274);
            n.param<double>("firstCobra_d1", d1, 0.062);
            n.param<double>("firstCobra_d4", d4, 0.257);
            n.param<double>("firstCobra_d6", d6, 0.07);
            n.param<double>("kSpeedResolution", speed_resolution, 0.01);
            n.param<double>("kRotateResolution", rotate_resolution, 0.4);
            N = N1 + N2;

            /*************算出机械臂各关节信息并输出到屏幕********/
            std::cout << "各电机角度，单位：度以及底层电机控制量速度，单位：rpm:" << std::endl;
            for (int i = 0; i < N; i++)
            {
                std::cout << "电机[" << i + 1 << "] Angle:" << fixed << setprecision(2) << setiosflags(ios::left) << setw(7) << pos[i] << "[" << setw(4) << LimitMin[i] << "~" << setw(3) << LimitMax[i] << "];";
                std::cout << "速度控制指令: " << setiosflags(ios::left) << setw(6) << firstCobraPubTosimMsg.firstSpeed[i] << ";";
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
            cout << "JointSpeedLevel:" << cobra_Vel_level << endl;
            cout << "\33[32mENDSpeedLevel:\033[0m" << end_mode_speed_level << "\33[32m   XYZ_speed:\033[0m" << speed_resolution * end_mode_speed_level << "\33[32m   RPY_speed:\033[0m" << rotate_resolution * end_mode_speed_level << endl;

            cout << endl;
            if (auto_mode) // 自主模式
            {
                cout << "\33[32m自主模式\033[0m" << endl;
                auto_arm();
            }
            else // 默认为遥控模式
            {
                if (endOrJointMode == JOINT_MODE) // 一般控制
                {
                    /*一键展开*/
                    if (flag_extend) // 一键展开  展开过程中不能松开按键 否则将停止展开   展开完毕需要松开“↓”和→
                    {
                        flag_need_set_ModeV = 1; // 待会需要设置速度模式
                        if (flag_need_set_ModeP) // 在展开循环中，只执行一次该循环，即只设置一次位置模式，否则会出现：设置位置模式→控制电机位置→设置位置模式→控制电机位置→...一直循环这样会使电机不动。
                        {
                            // firstCobraPubTosimMsg.firstPos[5] = POS1[5]/ (360.0 / Motor_Reduction_Ratio[5]);
                            for (int i = 0; i < N1; i++) // 将各个电机设置成位置模式
                            {
                                firstCobraPubTosimMsg.firstNeedMode = need_p;
                                firstCobraPubTosimMsg.firstPos[i] = pos_extend[i] / (360.0 / Motor_Reduction_Ratio[i]);
                                firstCobraPubTosimMsg.firstPosMode = 1;
                            }
                            flag_need_set_ModeP = 0;
                        }
                        firstCobraPubTosim_.publish(firstCobraPubTosimMsg);
                        cout << "\33[32m一键展开中！！！\033[0m" << endl;
                    }

                    /*一键还原*/
                    else if (flag_back) // 一键还原  还原过程中不能松开按键 否则将停止还原   还原完毕需要松开“↓”和→
                    {
                        for (int i = 0; i < N; i++)
                            cout << "pos_int[" << i << "]:" << pos_init[i] << endl;
                        flag_need_set_ModeV = 1; // 待会需要设置速度模式
                        if (flag_need_set_ModeP) // 在if(flag_back)循环中，只执行一次这个子循环，即只设置一次位置模式，否则会出现：设置位置模式→控制电机位置→设置位置模式→控制电机位置→...一直循环这样会使电机不动。
                        {
                            for (int i = 0; i < N1; i++) // 将各个电机设置成位置模式
                            {
                                firstCobraPubTosimMsg.firstNeedMode = need_p;
                                firstCobraPubTosimMsg.firstPos[i] = pos_init[i] / (360.0 / Motor_Reduction_Ratio[i]);
                                firstCobraPubTosimMsg.firstPosMode = 1;
                            }
                            flag_need_set_ModeP = 0;
                        }
                        firstCobraPubTosim_.publish(firstCobraPubTosimMsg);
                        cout << "\33[32m一键归位中！！！\033[0m" << endl;
                    }

                    else if (flagA) //  ps手柄的R2
                    {
                        flag_need_set_ModeV = 1; // 待会需要设置速度模式
                        if (flag_need_set_ModeP) // 在if(flag_back)循环中，只执行一次这个子循环，即只设置一次位置模式，否则会出现：设置位置模式→控制电机位置→设置位置模式→控制电机位置→...一直循环这样会使电机不动。
                        {
                            for (int i = 0; i < N1; i++) // 将各个电机设置成位置模式
                            {
                                firstCobraPubTosimMsg.firstNeedMode = need_p;
                                firstCobraPubTosimMsg.firstPos[i] = pos[i] / (360.0 / Motor_Reduction_Ratio[i]);
                                firstCobraPubTosimMsg.firstPosMode = 1;
                                if (i == 3)
                                    firstCobraPubTosimMsg.firstPos[i] = -82 / (360.0 / Motor_Reduction_Ratio[i]);
                                if (i == 4)
                                    firstCobraPubTosimMsg.firstPos[i] = 0 / (360.0 / Motor_Reduction_Ratio[i]);
                                if (i == 5)
                                    firstCobraPubTosimMsg.firstPos[i] = 0 / (360.0 / Motor_Reduction_Ratio[i]);
                            }
                            flag_need_set_ModeP = 0;
                        }
                        firstCobraPubTosim_.publish(firstCobraPubTosimMsg);
                        // cout <<"\33[32m一键归位中！！！\033[0m"<< endl;
                    }

                    else if (flagY) // PS手柄的R1
                    {
                        flag_need_set_ModeV = 1; // 待会需要设置速度模式
                        if (flag_need_set_ModeP) // 在if(flag_back)循环中，只执行一次这个子循环，即只设置一次位置模式，否则会出现：设置位置模式→控制电机位置→设置位置模式→控制电机位置→...一直循环这样会使电机不动。
                        {
                            for (int i = 0; i < N1; i++) // 将各个电机设置成位置模式
                            {
                                firstCobraPubTosimMsg.firstNeedMode = need_p;
                                firstCobraPubTosimMsg.firstPos[i] = pos[i] / (360.0 / Motor_Reduction_Ratio[i]);
                                firstCobraPubTosimMsg.firstPosMode = 1;
                                if (i == 3)
                                    firstCobraPubTosimMsg.firstPos[i] = 87 / (360.0 / Motor_Reduction_Ratio[i]);
                                if (i == 4)
                                    firstCobraPubTosimMsg.firstPos[i] = 0 / (360.0 / Motor_Reduction_Ratio[i]);
                                if (i == 5)
                                    firstCobraPubTosimMsg.firstPos[i] = 0 / (360.0 / Motor_Reduction_Ratio[i]);
                            }
                            flag_need_set_ModeP = 0;
                        }
                        firstCobraPubTosim_.publish(firstCobraPubTosimMsg);
                        // cout <<"\33[32m一键归位中！！！\033[0m"<< endl;
                    }

                    //                else if(flagX)//①
                    //                 {
                    //                     flag_need_set_ModeV = 1; //待会需要设置速度模式
                    //                     if(flag_need_set_ModeP)  //在if(flagX)循环中，只执行一次这个子循环，即只设置一次位置模式，否则会出现：设置位置模式→控制电机位置→设置位置模式→控制电机位置→...一直循环这样会使电机不动。
                    //                     {

                    //                         flag_need_set_ModeP = 0;
                    //                     }
                    //                     //设置位置

                    //                 }

                    /* ↓↓↓↓↓↓↓↓↓↓在此增加加一键模态↓↓↓↓↓↓模板模板模板↓↓↓↓↓↓↓↓↓↓↓
                     * else if (flagxxx)
                     * {
                     * flag_need_set_ModeV = 1; //待会需要设置速度模式
                         if(flag_need_set_ModeP)  //在if(flagxxx)循环中，只执行一次这个子循环，即只设置一次位置模式，否则会出现：设置位置模式→控制电机位置→设置位置模式→控制电机位置→...一直循环这样会使电机不动。
                         {
                             //设置位置模式


                             flag_need_set_ModeP = 0;
                         }
                         //设置位置
                     *
                     * }
                     *
                    else
                        ;
                     * ↑↑↑↑↑↑↑↑↑↑在此增加一键模态↑↑↑↑↑↑模板模板模板↑↑*/

                    /*位置控制完成，改回速度控制*/
                    if (flag_need_set_ModeV && !flag_pos_now) // 已经松开“↓”和→ 且需要开启速度模式  flag_need_set_ModeV只会在用过位置模式以后会变成1
                    {
                        std::cout << "速度模式已开启" << std::endl;
                        firstCobraPubTosimMsg.firstNeedMode = need_v;
                        firstCobraPubTosim_.publish(firstCobraPubTosimMsg);
                        flag_need_set_ModeV = 0;
                    }

                    /*************控制各电机速度********/
                    /**
                     * 0 大臂旋转
                     * 1 大臂前后
                     * 2 小臂上下
                     * 3 爪的上下
                     * 4 爪的左右
                     * 5 爪的上下
                     * 6 爪的张合
                     **/
                    for (int i = 0; i < N1; i++)
                    {
                        firstCobraPubTosimMsg.firstSpeed[i] = ((1 + Is_ChangeV[i] * (cobra_Vel_level - 1)) * flag[i] * cobra_motor_sign[i]) * (2 * 3.1415926) / 180; // 仿真环境下需要修改单位为 弧度每秒

                        // firstCobraPubTosimMsg.firstSpeed[i] = (1 + Is_ChangeV[i] * (cobra_Vel_level - 1)) * cobra_Vel[i] * flag[i] * cobra_motor_sign[i]; //发布速度单位为....转每分钟
                    }
                    firstCobraPubTosim_.publish(firstCobraPubTosimMsg); // 发布速度
                }

                else if (endOrJointMode == END_MODE) // 末端控制
                {
                    CartesianToJoint(handOrBaseMode); // 解算速度

                    if (flag_d_theta_clear) // 把各电机速度清零
                        for (int i = 1; i < N1; i++)
                        {
                            dtheta[i] = 0;
                        }
                    if (handOrBaseMode) // 显示出末端坐标系手掌or基座
                        std::cout << "\33[32m基座\033[0m坐标系控制：";
                    else
                        std::cout << "\33[32m手掌\033[0m坐标系控制：";
                    std::cout << std::endl;
                    std::cout << "dx:" << dx << ";dy:" << dy << ";dz:" << dz << ";dpitch:" << dpitch << ";dyaw:" << dyaw << ";droll:" << droll << ";" << std::endl;

                    for (int i = 0; i < N1; i++)
                        firstCobraPubTosimMsg.firstSpeed[i] = DH_sign[i] * dtheta[i + 1] / 5; // 仿真环境下需要修改单位为弧度每秒
                    // firstCobraPubTosimMsg.firstSpeed[i] = DH_sign[i] * dtheta[i + 1] * Motor_Reduction_Ratio[i] * 60.0 / (3.1415926 * 2);
                    firstCobraPubTosim_.publish(firstCobraPubTosimMsg);
                }

                // publish

                for (int i = 0; i < N; i++)
                {
                    timon_angle.arm[i] = pos[i];
                }
                cobra_angle_pub_.publish(timon_angle);

                std::cout << "  a3:  " << a3 << std::endl;
                std::cout << "  d1:  " << d1 << "  d4:  " << d4 << "  d6:  " << d6 << std::endl;
                if (endOrJointMode == JOINT_MODE)
                    cout << "\33[32m关节控制\033[0m" << endl;
                if (endOrJointMode == END_MODE)
                    cout << "\33[32m末端控制\033[0m" << endl;
            }
        }
        for (int i = 0; i <= 20; i++) // 清屏
        {
            printf(cursup);
            printf(cursclean);
        }
    }

private:
    ros::Subscriber joy_sub_;
    // ros::Subscriber control_sub_;
    ros::Subscriber firstCobraSubFromsim_;

    ros::Publisher cobra_angle_pub_;
    // ros::Publisher cobra_to_rviz_pub_;
    ros::Publisher firstCobraPubTosim_;

    ros::Timer controller;

    bool now_control = true; // 仿真环境下直接设置为cobra模式
    bool cobra = true;
    bool auto_mode = false;
    Matrix4d RHS;
    Matrix4d input_goal;
    int flag_auto = 1;

    float flag_button = 0;
    int cobra_Vel_level = 1; // 机械臂单机速度档位
    int end_mode_speed_level = 2;
    int flag_extend = 0;           // 一键展开
    int flag_need_set_ModeV = 0;   // 需要设置成速度模式
    int flag_need_set_ModeP = 0;   // 需要设置成位置模式
    int flag_back = 0;             // 一键归位
    int flag_end_stop = 0;         // 末端控制停止标志符
    int endOrJointMode = END_MODE; // 控制模式，1为一般控制，2为末端控制
    int flag_pos_now = 0;          // 现在正在位置控制  注意：凡是用到位置模式都需要将该标志位改为1；
    bool All_Motor_IsOnlineAndEnable = true;
    int flag_d_theta_clear = 1;
    bool handOrBaseMode = false;
    // 机械臂参数
    double a2, a3, a4, d1, d4, d6, a6;
    double dx = 0, dy = 0, dz = 0, dpitch = 0, dyaw = 0, droll = 0, dax = 0, day = 0, daz = 0;
    double dnx = 0, dny = 0, dnz = 0, dpx = 0, dpy = 0, dpz = 0, dox = 0, doy = 0, doz = 0;
    double roll, pitch, yaw;
    double dtheta[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 末端控制状态控制速度 ，下标从1开始，theta[0]无用。单位弧度。
    double theta[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};  // 末端控制状态角度 ，下标从1开始，theta[0]无用。单位弧度。

    double nx, ox, ax, px, ny, oy, ay, py, nz, oz, az, pz;
    int flagX = 0, flagY = 0, flagA = 0, flagB = 0, flagAB = 0, flagAX = 0, flagXY = 0, flagBY = 0; // 八种位置模式组合键

    nubot_msgs::simSub firstCobraPubTosimMsg;
    // nubot_msgs::innfosSub firstCobraPubTosimMsg;
    // nubot_msgs::cobra3_angle_position cobra3_angle; // need change
    // nubot_msgs::cobraToRviz cobraToRviz;
    nubot_msgs::angle_position timon_angle;

    /**
    ↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓参数调节↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓
    各数组序号
    * 0 大臂旋转
    * 1 大臂前后
    * 2 小臂上下
    * 3 爪的上下
    * 4 爪的左右
    * 5 爪的旋转
    * 6 爪的开合
    **/
    float flag[7] = {0, 0, 0, 0, 0, 0, 0};                                // 普通遥控各电机遥控指令
    double flag2[6] = {0, 0, 0, 0, 0, 0};                                 // 末端遥控各电机指令  1:d_x   2:d_z  3:d_pitch   4:d_yaw
    double pos[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};       // 各电机位置信息 单位：°
    double pos2theta[15] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 当前位置到末端控制状态角转换差值    仿真环境
    int LimitMax[6] = {180, 85, 30, 300, 120, 60};                        // 各电机最大限位 单位：° 度
    int LimitMin[6] = {-180, -42, -110, -300, -120, -145};                // 各电机最小限位 单位：° 度

    int DH_sign[6] = {1, 1, 1, 1, 1, 1};                     // 电机摆放对DH建模正方向的影响  仿真环境
    int cobra_motor_sign[6] = {1, 1, 1, 1, 1, 1};            // 电机与遥控器正反转适配     仿真环境
    int Motor_Reduction_Ratio[6] = {36, 36, 36, 36, 36, 36}; // 各电机减速比 仿真环境

    // 机械臂速度： 6 * cobra_Vel/Motor_Reduction_Ratio 度/秒
    double cobra_Vel[6] = {168, 130, 130, 90, 90, 180}; // 各电机基础速度   此速度为基础电机每分钟多少转   具体到机械臂角度计算： cobra_Vel/60 * 360/Motor_Reduction_Ratio = 6 * cobra_Vel/Motor_Reduction_Ratio 度/秒
    // int Motor_Reduction_Ratio[6] = {101, 101, 101, 36, 36, 36}; //各电机减速比

    int Motor_PosV[6] = {350, 350, 50, 200, 500, 500}; // 位置模式时，各电机转速
    int cobra_Vel_Total_level = 3;                     // 机械臂速度档位数， 成倍增加，即3档是1档的3倍速，2档是1档的2倍速
    int end_mode_Total_level = 3;
    int Is_ChangeV[6] = {1, 1, 1, 1, 1, 1};       // 各电机是否受加减速影响，1为可以加减速，0为屏蔽加减速。
    float pos_extend[6] = {0, 65, 60, -21, 0, 0}; // 机械臂展开角度，单位° 度
    bool flag_init = true;
    double pos_init[13] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}; // 一键归位用，若想手动设置归位角度将上一行的 flag_init设置为false
    float temperature[12] = {0, 0, 0, 0, 0, 0};                    // 电机的温度
    float DianLiu[12] = {0, 0, 0, 0, 0, 0};                        // 电机的dianliu
    int endV = 50;                                                 // 末端三个关节的一键模式的速度
    double cobra3_pos[6] = {0, 0, 0, 0, 0, 0};
    // ↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑参数调节↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑↑
    int i;
    int need_v = 1;                 // 标志位，程序中不应改变
    int need_p = 2;                 // 标志位，
    double rotate_resolution = 0.2; // 末端旋转速度的分辨率（越大速度越快）
    double speed_resolution = 0.05; // 末端平移速度的分辨率（越大速度越快）

    float POS1[6] = {0, 0, 0, 0, 0, 100};
    int N1 = 6, N2 = 0, N = N1 + N2;
    bool enableOK = false;
};
int main(int argc, char *argv[])
{
    /*******************************ROS*****************************/
    ros::init(argc, argv, "cc_7RL3_sim");

    nubot_arm_node2 arm;

    ros::NodeHandle n;

    ros::spin();
    return 0;
}
