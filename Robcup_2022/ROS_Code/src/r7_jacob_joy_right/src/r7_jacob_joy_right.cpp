#include <ros/ros.h>
#include <time.h>
#include <signal.h>
#include <string.h>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <vector>
#include <iostream>
#include <thread>
#include <eigen3/Eigen/Dense>

#include "std_msgs/Float64MultiArray.h"
#include "nubot_msgs/angle_position.h"
#include "nubot_msgs/innfosPub.h"
#include "nubot_msgs/innfosSub.h"
// #include "nubot_msgs/simPub.h"
// #include "nubot_msgs/simSub.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

#define cursup "\033[A"
#define cursclean "\033[2K"
#define curshome "\033[0;0H"

#define JOINT_MODE 1
#define END_MODE 2

using namespace std;
using namespace Eigen;
using namespace chrono;

high_resolution_clock::time_point control_ts, control_te, t3, t4;
duration<double> time_span;
double ret;

void Clear(void)
{
    control_ts = high_resolution_clock::now();
    control_te = control_ts;
}

double GetValue(void)
{

    control_te = high_resolution_clock::now();
    duration<double> time_span = duration_cast<duration<double>>(control_te - control_ts);
    ret = time_span.count();
    return ret;
}

class nubot_arm_node2
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    nubot_arm_node2()
    {

        ros::NodeHandle n;
        /*********节点*********/
        arm_param_init();
        joy_sub_ = n.subscribe("/joy", 1, &nubot_arm_node2::joyCallback, this);
        controller = n.createTimer(ros::Duration(1.0 / 100.0), &nubot_arm_node2::timeController, this, false);
        cobra_angle_pub_ = n.advertise<nubot_msgs::angle_position>("timon_angle", 1);
        secondCobraPubToInnfos_ = n.advertise<nubot_msgs::innfosSub>("innfosSub_right", 1, true);
        secondend_pos_d_ = n.advertise<std_msgs::Float64MultiArray>("/end_pos_d_r", 1, true);
        secondend_pos_fb_ = n.advertise<std_msgs::Float64MultiArray>("/end_pos_fb_r", 1, true);
        secondend_vel_d_ = n.advertise<std_msgs::Float64MultiArray>("/end_vel_d_r", 1, true);
        secondend_vel_fb_ = n.advertise<std_msgs::Float64MultiArray>("/end_vel_fb_r", 1, true);
        RHS_trans_ = n.advertise<std_msgs::Float64MultiArray>("/RHS_trans_r", 1, true);
        secondend_FT_ = n.advertise<std_msgs::Float64MultiArray>("/end_FT_r", 1, true);

        secondCobraSubFromInnfos_ = n.subscribe("innfosPub", 1, &nubot_arm_node2::secondCobraCallback, this);
        secondCobraPubToInnfosMsg.secondNeedMode = need_v;
    }

    ~nubot_arm_node2() // 关闭节点
    {
        cout << "Node closing！ Begin initializing" << endl;
    }
    void secondCobraCallback(const nubot_msgs::innfosPub &secondCobra)
    {
        for (int i = 0; i < N2; i++)
        {
            pos[i] = secondCobra.secondOriginalPos[i];            // 单位：转
            pos[i] = (360.0 / Motor_Reduction_Ratio[i]) * pos[i]; // 除以减速比乘以360转化成目视角度  计算用的pos不能取round
            theta[i] = double(M_PI * pos[i] * DH_sign[i] / 180);  // 得到姿态角度，单位：弧度
            if (flag_init)                                        // online后第一次读取角度数据给它存起来，且开一次节点仅存一次，方便臂的归位
            {
                pos_init[i] = pos[i];
                arm_param_init();
            }
        }
        flag_init = false;
        for (int i = 0; i < N2; i++)
        {
            temperature[i] = secondCobra.innfosTemperature[i + N1];
            DianLiu[i] = secondCobra.innfosCurrent[i + N1];
            joint_vel[i] = DH_sign[i] * (M_PI * 2 / Motor_Reduction_Ratio[i]) * secondCobra.secondOriginalVel[i] / 60; // 除以减速比乘以2*M_PI/60转化成弧度/s
            if (abs(joint_vel[i]) < 0.005)
                joint_vel[i] = 0;
        }
        for (int i = 0; i < 6; i++)
            secondend_FT.data[i] = secondCobra.secondEndFS[i];
        enableOK = secondCobra.enableOK;
    }

    void CartesianToJoint(bool mode)
    {
        Matrix4d gg_;    // 过渡矩阵
        Vector4d cacul_; // 临时计算用向量
        cacul_.setZero();
        gg_.setIdentity();
        Matrix<Matrix4d, 1, Dynamic> A__, J__;
        A__.resize(N2);
        J__.resize(N2);
        RHS.setZero();
        for (int i = 0; i < N2; i++)
        {
            A__(i) = Matrix4d::Identity(); // 初始化A0~A6为4*4单位矩阵
            J__(i) = Matrix4d::Identity(); // 初始化A0~A6为4*4单位矩阵
        }

        // 求RHS与jacob 先Z后X
        const float er = 0.0000001; // 三角函数计算精度
        for (int i = 0; i < N2; i++)
        {
            A__(i) << round(cos(theta(i) + DH_bias(i)) / er) * er, round(-cos(DH_alpha(i)) * sin(theta(i) + DH_bias(i)) / er) * er, round(sin(theta(i) + DH_bias(i)) * sin(DH_alpha(i)) / er) * er, DH_a(i) * round(cos(theta(i) + DH_bias(i)) / er) * er,
                round(sin(theta(i) + DH_bias(i)) / er) * er, round(cos(theta(i) + DH_bias(i)) * cos(DH_alpha(i)) / er) * er, round(-cos(theta(i) + DH_bias(i)) * sin(DH_alpha(i)) / er) * er, DH_a(i) * round(sin(theta(i) + DH_bias(i)) / er) * er,
                0, round(sin(DH_alpha(i)) / er) * er, round(cos(DH_alpha(i)) / er) * er, DH_d(i),
                0, 0, 0, 1;
        }

        for (int i = 0; i < N2; i++)
        {
            for (int j = N2 - 1; j >= i; j--)
            {
                J__(i) = A__(j) * J__(i);
            }
        }
        RHS = J__(0);
        gg_ = guodu_ * RHS;
        local_attitude = gg_.topLeftCorner<3, 3>();
        local_position << RHS(0, 3), RHS(1, 3), RHS(2, 3), RHS(3, 3);
        local_position = guodu_ * local_position; // 转换到 以上为x，右为y，前为z的基座标系，同时，以x，y各加上圆周运动的半径为目标点，进行速度方向切换

        for (int i = 0; i < 3; i++)
        {
            RHS_trans.data[i] = local_attitude(0, i);
            RHS_trans.data[i + 3] = local_attitude(1, i);
            RHS_trans.data[i + 6] = local_attitude(2, i);
        }
        RHS_trans_.publish(RHS_trans);
        cout << "local_attitude is" << endl
             << local_attitude << endl;

        for (int i = 0; i < N2; i++)
        {
            Jacob(0, i) = -J__(i)(0, 0) * J__(i)(1, 3) + J__(i)(1, 0) * J__(i)(0, 3); //-nx*py+ny*px
            Jacob(1, i) = -J__(i)(0, 1) * J__(i)(1, 3) + J__(i)(1, 1) * J__(i)(0, 3); //-ox*py+oy*px
            Jacob(2, i) = -J__(i)(0, 2) * J__(i)(1, 3) + J__(i)(1, 2) * J__(i)(0, 3); //-ax*py+ay*px
            Jacob(3, i) = J__(i)(2, 0);                                               // nz
            Jacob(4, i) = J__(i)(2, 1);                                               // oz
            Jacob(5, i) = J__(i)(2, 2);                                               // az
        }
        MatrixXd CC__ = Jacob * Jacob.transpose() + MatrixXd::Identity(6, 6);
        // cout << "Jacob * Jacob.transpose() is" << endl
        //      << Jacob * Jacob.transpose() << endl;
        // cout << "CC__ is" << endl
        //      << CC__ << endl;
        // cout << "Jacob 奇异鲁棒性逆 is" << endl;
        // cout << Jacob.transpose() * pinv_eigen_based(CC__) << endl; // 奇异鲁棒性逆

        FbEndmove = Jacob * joint_vel;                         // 末端坐标系下反馈速度
        cacul_ << FbEndmove(0), FbEndmove(1), FbEndmove(2), 1; // 将末端坐标系下的平移反馈速度转换到基座坐标系下
        FbEndmove.block<3, 1>(0, 0) = (cacul_.transpose() * gg_.inverse()).block<1, 3>(0, 0);

        end_move << dx, dy, dz, rx, ry, rz;
        if (mode)
        {
            // end_move = basemove_to_end(RHS, end_move);
            cacul_ << end_move(0), end_move(1), end_move(2), 1; // 将基座坐标系下的期望平移速度转换到末端坐标系下用雅克比矩阵求解
            cacul_ = cacul_.transpose() * gg_;
            for (int i = 0; i < 3; i++)
                end_move[i] = cacul_[i];
            // dtheta = Jacob.transpose() * pinv_eigen_based(CC__) * end_move_desire;//奇异鲁棒逆用于保证位置跟踪speed_resolution0.2，rotate_resolution0.2
            dtheta = pinv_eigen_based(Jacob) * end_move; // 伪逆用于末端速度计算
        }
        else
        {
            // dtheta = Jacob.transpose() * pinv_eigen_based(CC__) * end_move_desire;//奇异鲁棒逆用于保证位置跟踪speed_resolution0.2，rotate_resolution0.2
            dtheta = pinv_eigen_based(Jacob) * end_move; // 伪逆用于末端速度计算
        }
        // cout << "dtheta is" << endl
        //      << dtheta << endl;

        // 速度过快
        for (int i = 0; i < N2; i++)
        {
            if (abs(dtheta(i)) > 1.201)
            {
                cout << "dtheta[" << i << "]:" << dtheta(i) << "!!!!!!!!!" << endl;
                dtheta.setZero();
                cout << "dtheta is" << endl
                     << dtheta << endl;
                break;
            }
        }
    }

    // 利用Eigen库，采用SVD分解的方法求解矩阵逆或者伪逆，默认误差er为0
    Eigen::MatrixXd pinv_eigen_based(Eigen::MatrixXd &origin, const float err = 0.0001)
    {
        // 进行svd分解
        Eigen::JacobiSVD<Eigen::MatrixXd> svd_holder(origin, Eigen::ComputeThinU | Eigen::ComputeThinV);
        // 构建SVD分解结果
        Eigen::MatrixXd U = svd_holder.matrixU();
        Eigen::MatrixXd V = svd_holder.matrixV();
        Eigen::MatrixXd D = svd_holder.singularValues();
        // cout<<"D :\n"<<D<<endl;
        //  构建S矩阵
        Eigen::MatrixXd S(V.cols(), U.cols());
        S.setZero();

        for (unsigned int i = 0; i < D.size(); ++i)
        {
            if (D(i, 0) > err)
            {
                S(i, i) = 1 / D(i, 0);
            }
            else
            {
                S(i, i) = 0;
            }
        }
        // pinv_matrix = V * S * U^T
        return V * S * U.transpose();
    }
    // 将相对于基座的运动通过算子转化到末端坐标系下的运动(关系：末端运动算子=RHS*基座运动算子*RHS^-1)，输入RHS与end_move，输出一个相同的向量并赋值给end_move
    VectorXd basemove_to_end(Matrix4d &rhs, VectorXd &base_move)
    {
        Matrix4d CC_;
        CC_.setZero();
        CC_ << 0, -base_move(5), base_move(4), base_move(0),
            base_move(5), 0, -base_move(3), base_move(1),
            -base_move(4), base_move(3), 0, base_move(2),
            0, 0, 0, 0;
        CC_ = rhs * CC_ * rhs.inverse();
        VectorXd AA_;
        AA_.setZero(6);
        AA_ << CC_(0, 3), CC_(1, 3), CC_(2, 3), CC_(2, 1), CC_(0, 2), CC_(1, 0);

        return AA_;
    }
    /***************handle joy input***************/
    void joyCallback(const sensor_msgs::Joy &joy)
    {
        /*********↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓机械臂PS4手柄控制↓↓↓↓↓↓↓↓↓↓↓↓↓↓↓*********/
        if (now_control == cobra)
        {
            if (joy.axes[6] == 1) // 十字键←， 左右臂控制循环切换，1为双臂，2为左臂，3为右臂
            {
                left_or_right = left_or_right + 1;
                if (left_or_right > 3)
                    left_or_right = 1;
            }
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
                    n.setParam("secondCobra_d6", 0.512);
                    n.setParam("secondCobra_a6", 0.00);
                }
                if (joy.buttons[0] == 1) // 爪子末端
                {
                    n.setParam("secondCobra_d6", 0.284);
                    n.setParam("secondCobra_a6", 0.00);
                }
                if (joy.buttons[5] == 1) // 开门末端
                {
                    n.setParam("secondCobra_d6", 0.284);
                    n.setParam("secondCobra_a6", 0.15);
                }
            }

            if (joy.axes[6] != -1 || joy.axes[7] != -1) // 松掉下或右
            {
                secondCobraPubToInnfosMsg.secondNeedMode = need_v; // 设置速度模式
                secondCobraPubToInnfos_.publish(secondCobraPubToInnfosMsg);
                flag_need_set_ModeV = 0;
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
                    rx = 0;
                    ry = 0;
                    rz = 0;
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
                    rx = rotate_resolution * end_mode_speed_level * flag2[4];

                    ry = rotate_resolution * end_mode_speed_level * flag2[3];

                    if (joy.axes[5] != 0)
                        dx = speed_resolution * end_mode_speed_level * (joy.buttons[5] + (joy.axes[5] - 1) / 2); // 向上为正  上下

                    if (joy.axes[7] != 1)
                        rz = rotate_resolution * end_mode_speed_level * (joy.buttons[1] - joy.buttons[3]);

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
            // cout << "按键释放" << endl;
            for (int i = 0; i < N2; i++)
            {
                flag[i] = 0;
                dtheta.setZero();
            }
            for (int i = 0; i < 6; i++)
            {
                flag2[i] = 0;
            }
            flag_d_theta_clear = 1;
        }
    }

    /***************时间中断，控制部分***************/
    void timeController(const ros::TimerEvent &e)
    {
        auto start = system_clock::now();
        if (!enableOK)
            cout << "\033[47;31m电机未初始化好!!!请等待！！！\033[0m" << endl
                 << "\033[47;31m很久没好则为网络（网线）未连接好or机械臂未上电！！！\033[0m" << endl;
        else
        {
            ros::NodeHandle n;
            n.param<int>("cobraNum1", N1, 7);
            n.param<int>("cobraNum2", N2, 7);
            // 机械臂参数
            n.param<double>("kSpeedResolution", speed_resolution, 0.02);
            n.param<double>("kRotateResolution", rotate_resolution, 0.1);

            n.getParam("secondLimitMax", LimitMax);
            n.getParam("secondLimitMin", LimitMin);

            /*************算出机械臂各关节信息并输出到屏幕********/
            cout << "\033[47;31m右臂\033[0m各电机角度，单位：度以及底层电机控制量速度，单位：rpm:" << endl;
            for (int i = 0; i < N2; i++)
            {
                cout << "电机[" << i + 1 << "] Angle:" << fixed << setprecision(2) << setiosflags(ios::left) << setw(7) << pos[i] << "[" << setw(4) << LimitMin[i] << "~" << setw(3) << LimitMax[i] << "];";
                // cout << "[";
                // cout << setw(4) << LimitMin.at(i);
                // cout << "~";
                // cout << setw(4) << LimitMax.at(i);
                // cout << "];";
                cout << "速度控制指令: " << setiosflags(ios::left) << setw(6) << secondCobraPubToInnfosMsg.secondSpeed[i] << ";";
                if (endOrJointMode == JOINT_MODE)
                {
                    cout << "flag[" << i << "]:" << flag[i] << ";" << endl;
                }
                else if (endOrJointMode == END_MODE)
                {
                    cout << "dtheta" << i << "：" << dtheta[i] << ";" << endl;
                }
            }
            cout << endl;
            cout << "Current:";
            for (int i = 0; i < N2; i++)
                cout << "[" << round(100 * DianLiu[i]) / 100 << "]"
                     << ";";
            cout << endl;

            cout << "Temperature:";
            for (int i = 0; i < N2; i++)
                cout << temperature[i] << ";";
            cout << endl;

            cout << "JointSpeedLevel:" << cobra_Vel_level << endl;
            cout << "\33[32mENDSpeedLevel:\033[0m" << end_mode_speed_level << "\33[32m   XYZ_speed:\033[0m" << speed_resolution * end_mode_speed_level << "\33[32m   RPY_speed:\033[0m" << rotate_resolution * end_mode_speed_level << endl;
            cout << "joint_vel[1]" << joint_vel[0] << ";joint_vel[2]" << joint_vel[1] << ";joint_vel[3]" << joint_vel[2] << ";joint_vel[4]" << joint_vel[3] << ";joint_vel[5]" << joint_vel[4] << ";joint_vel[6]" << joint_vel[5] << ";joint_vel[7]" << joint_vel[6] << endl;

            cout << endl;
            if (left_or_right == 3 || left_or_right == 1)
            {

                if (endOrJointMode == JOINT_MODE) // 一般控制
                {
                    /*一键展开*/
                    if (flag_extend) // 一键展开  展开过程中不能松开按键 否则将停止展开   展开完毕需要松开“↓”和→
                    {
                        flag_need_set_ModeV = 1; // 待会需要设置速度模式
                        if (flag_need_set_ModeP) // 在展开循环中，只执行一次该循环，即只设置一次位置模式，否则会出现：设置位置模式→控制电机位置→设置位置模式→控制电机位置→...一直循环这样会使电机不动。
                        {
                            // secondCobraPubToInnfosMsg.secondPos[5] = POS1[5]/ (360.0 / Motor_Reduction_Ratio[5]);
                            for (int i = 0; i < N2; i++) // 将各个电机设置成位置模式
                            {
                                secondCobraPubToInnfosMsg.secondNeedMode = need_p;
                                secondCobraPubToInnfosMsg.secondPos[i] = pos_extend[i] / (360.0 / Motor_Reduction_Ratio[i]);
                                secondCobraPubToInnfosMsg.secondPosMode = 1;
                            }
                            flag_need_set_ModeP = 0;
                        }
                        secondCobraPubToInnfos_.publish(secondCobraPubToInnfosMsg);
                        cout << "\33[32m一键展开中！！！\033[0m" << endl;
                    }

                    /*一键还原*/
                    else if (flag_back) // 一键还原  还原过程中不能松开按键 否则将停止还原   还原完毕需要松开“↓”和→
                    {
                        for (int i = 0; i < N2; i++)
                            cout << "pos_int[" << i << "]:" << pos_init[i] << endl;
                        flag_need_set_ModeV = 1; // 待会需要设置速度模式
                        if (flag_need_set_ModeP) // 在if(flag_back)循环中，只执行一次这个子循环，即只设置一次位置模式，否则会出现：设置位置模式→控制电机位置→设置位置模式→控制电机位置→...一直循环这样会使电机不动。
                        {
                            ////////////////////////以下是在一键还原过程中插入先还原3号关节这一流程，防止直接还原的碰撞///////////////////////////////
                            // secondCobraPubToInnfosMsg.secondNeedMode = need_p;
                            // secondCobraPubToInnfosMsg.secondPos[0] = pos[0] / (360.0 / Motor_Reduction_Ratio[0]);
                            // secondCobraPubToInnfosMsg.secondPos[1] = pos[1] / (360.0 / Motor_Reduction_Ratio[1]);
                            // secondCobraPubToInnfosMsg.secondPos[2] = pos_init[2] / (360.0 / Motor_Reduction_Ratio[2]);
                            // secondCobraPubToInnfosMsg.secondPos[3] = pos[3] / (360.0 / Motor_Reduction_Ratio[3]);
                            // secondCobraPubToInnfosMsg.secondPos[4] = pos[4] / (360.0 / Motor_Reduction_Ratio[4]);
                            // secondCobraPubToInnfosMsg.secondPos[5] = pos[5] / (360.0 / Motor_Reduction_Ratio[5]);
                            // secondCobraPubToInnfosMsg.secondPos[6] = pos[6] / (360.0 / Motor_Reduction_Ratio[6]);
                            // secondCobraPubToInnfosMsg.secondPosMode = 1;
                            // secondCobraPubToInnfos_.publish(secondCobraPubToInnfosMsg);
                            ////////////////////////以上是在一键还原过程中插入先还原3号关节这一流程，防止直接还原的碰撞///////////////////////////////
                            for (int i = 0; i < N2; i++) // 将各个电机设置成位置模式
                            {
                                secondCobraPubToInnfosMsg.secondNeedMode = need_p;
                                secondCobraPubToInnfosMsg.secondPos[i] = pos_init[i] / (360.0 / Motor_Reduction_Ratio[i]);
                                secondCobraPubToInnfosMsg.secondPosMode = 1;
                            }
                            flag_need_set_ModeP = 0;
                        }
                        secondCobraPubToInnfos_.publish(secondCobraPubToInnfosMsg);
                        cout << "\33[32m一键归位中！！！\033[0m" << endl;
                    }

                    else if (flagA) //  ps手柄的R2
                    {
                        flag_need_set_ModeV = 1; // 待会需要设置速度模式
                        if (flag_need_set_ModeP) // 在if(flag_back)循环中，只执行一次这个子循环，即只设置一次位置模式，否则会出现：设置位置模式→控制电机位置→设置位置模式→控制电机位置→...一直循环这样会使电机不动。
                        {
                            for (int i = 0; i < N2; i++) // 将各个电机设置成位置模式
                            {
                                secondCobraPubToInnfosMsg.secondNeedMode = need_p;
                                secondCobraPubToInnfosMsg.secondPos[i] = pos[i] / (360.0 / Motor_Reduction_Ratio[i]);
                                secondCobraPubToInnfosMsg.secondPosMode = 1;
                                if (i == 3)
                                    secondCobraPubToInnfosMsg.secondPos[i] = -82 / (360.0 / Motor_Reduction_Ratio[i]);
                                if (i == 4)
                                    secondCobraPubToInnfosMsg.secondPos[i] = 0 / (360.0 / Motor_Reduction_Ratio[i]);
                                if (i == 5)
                                    secondCobraPubToInnfosMsg.secondPos[i] = 0 / (360.0 / Motor_Reduction_Ratio[i]);
                            }
                            flag_need_set_ModeP = 0;
                        }
                        secondCobraPubToInnfos_.publish(secondCobraPubToInnfosMsg);
                        // cout <<"\33[32m一键归位中！！！\033[0m"<< endl;
                    }

                    else if (flagY) // PS手柄的R1
                    {
                        flag_need_set_ModeV = 1; // 待会需要设置速度模式
                        if (flag_need_set_ModeP) // 在if(flag_back)循环中，只执行一次这个子循环，即只设置一次位置模式，否则会出现：设置位置模式→控制电机位置→设置位置模式→控制电机位置→...一直循环这样会使电机不动。
                        {
                            for (int i = 0; i < N2; i++) // 将各个电机设置成位置模式
                            {
                                secondCobraPubToInnfosMsg.secondNeedMode = need_p;
                                secondCobraPubToInnfosMsg.secondPos[i] = pos[i] / (360.0 / Motor_Reduction_Ratio[i]);
                                secondCobraPubToInnfosMsg.secondPosMode = 1;
                                if (i == 3)
                                    secondCobraPubToInnfosMsg.secondPos[i] = 87 / (360.0 / Motor_Reduction_Ratio[i]);
                                if (i == 4)
                                    secondCobraPubToInnfosMsg.secondPos[i] = 0 / (360.0 / Motor_Reduction_Ratio[i]);
                                if (i == 5)
                                    secondCobraPubToInnfosMsg.secondPos[i] = 0 / (360.0 / Motor_Reduction_Ratio[i]);
                            }
                            flag_need_set_ModeP = 0;
                        }
                        secondCobraPubToInnfos_.publish(secondCobraPubToInnfosMsg);
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
                        cout << "速度模式已开启" << endl;
                        secondCobraPubToInnfosMsg.secondNeedMode = need_v;
                        secondCobraPubToInnfos_.publish(secondCobraPubToInnfosMsg);
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
                    for (int i = 0; i < N2; i++)
                    {
                        // secondCobraPubToInnfosMsg.secondSpeed[i] = ((1 + Is_ChangeV[i] * (cobra_Vel_level - 1)) * flag[i] * cobra_motor_sign[i]) * (2 * M_PI) / 180; // 仿真环境下需要修改单位为 弧度每秒
                        secondCobraPubToInnfosMsg.secondSpeed[i] = (1 + Is_ChangeV[i] * (cobra_Vel_level - 1)) * cobra_Vel[i] * flag[i] * cobra_motor_sign[i]; // 发布速度单位为....转每分钟
                    }
                    secondCobraPubToInnfos_.publish(secondCobraPubToInnfosMsg); // 发布速度
                }

                else if (endOrJointMode == END_MODE) // 末端控制
                {
                    // if (abs(dx) + abs(dy) + abs(dz) + abs(rz) + abs(ry) + abs(rx) <= 0.05)
                    // {
                    //     cobraToRviz.end_speed_too_fast = false; // rviz关闭显示速度过快
                    //     cobra_to_rviz_pub_.publish(cobraToRviz);
                    // }
                    CartesianToJoint(handOrBaseMode); // 解算速度

                    if (flag_d_theta_clear) // 把各电机速度清零
                    {
                        dtheta.setZero();
                    }

                    if (handOrBaseMode) // 显示出末端坐标系手掌or基座
                        cout << "\33[32m基座\033[0m坐标系控制：";
                    else
                        cout << "\33[32m手掌\033[0m坐标系控制：";
                    cout << endl;
                    cout << "dx:" << dx << ";dy:" << dy << ";dz:" << dz << ";rx:" << rx << ";ry:" << ry << ";rz:" << rz << ";" << endl;

                    for (int i = 0; i < 6; i++)
                    {
                        secondend_vel_fb.data[i] = FbEndmove[i];
                    }
                    secondend_vel_d.data[0] = dx;
                    secondend_vel_d.data[1] = dy;
                    secondend_vel_d.data[2] = dz;
                    secondend_vel_d.data[3] = rx;
                    secondend_vel_d.data[4] = ry;
                    secondend_vel_d.data[5] = rz;
                    secondend_vel_d_.publish(secondend_vel_d);
                    secondend_vel_fb_.publish(secondend_vel_fb);

                    std::cout << setw(4) << "FBdx:" << FbEndmove[0] << ";FBdy:" << FbEndmove[1] << ";FBdz:" << FbEndmove[2] << ";FBrx:" << FbEndmove[3] << ";FBry:" << FbEndmove[4] << ";FBrz:" << FbEndmove[5] << ";" << std::endl;

                    cout << "相对于身体前方基坐标系坐标X:" << setprecision(4) << local_position(0) << ";Y:" << local_position(1) << ";Z:" << local_position(2) << ";" << endl;
                    for (int i = 0; i < N2; i++)
                        // secondCobraPubToInnfosMsg.secondSpeed[i] = DH_sign[i] * dtheta[i]; // 仿真环境下需要修改单位为弧度每秒
                        secondCobraPubToInnfosMsg.secondSpeed[i] = DH_sign[i] * dtheta[i] * Motor_Reduction_Ratio[i] * 60.0 / (3.1415926 * 2);
                    secondCobraPubToInnfos_.publish(secondCobraPubToInnfosMsg);
                }
            }
            secondend_FT_.publish(secondend_FT);
            if (endOrJointMode == JOINT_MODE)
                cout << "\33[32m关节控制\033[0m" << endl;
            if (endOrJointMode == END_MODE)
                cout << "\33[32m末端控制\033[0m" << endl;
            if (left_or_right == 1)
                cout << "\33[32m双臂控制\033[0m" << endl;
            if (left_or_right == 2)
                cout << "\33[32m左臂控制\033[0m" << endl;
            if (left_or_right == 3)
                cout << "\33[32m右臂控制\033[0m" << endl;
        }

        auto end = system_clock::now();
        auto duration = duration_cast<microseconds>(end - start);
        cout << "control loop spent" << setprecision(7) << double(duration.count()) * microseconds::period::num / microseconds::period::den << "second" << endl;
        for (int i = 0; i <= 30; i++) // 清屏
        {
            printf(cursup);
            printf(cursclean);
        }
        // cobraToRviz.endOrJointMode = endOrJointMode;
        // cobraToRviz.handOrBaseMode = handOrBaseMode;
        // cobraToRviz.joint_speed_level = cobra_Vel_level;
        // cobraToRviz.end_speed_level = end_mode_speed_level;
        // cobra_to_rviz_pub_.publish(cobraToRviz);
        // for (int i = 0; i < N2; i++)
        // {
        //     timon_angle.arm[i] = pos[i];
        // }
        // cobra_angle_pub_.publish(timon_angle);
    }

    void arm_param_init() // 变量初始化 *(a.m_storage.m_data)@9 在调试中查看矩阵或者向量中的值
    {
        Jacob = MatrixXd::Zero(6, N1);
        RHS.setZero();
        f_a_guodu_.setZero();
        guodu_.setZero();
        // 以上为x，右为y，前为z的基座标系（机器人身体坐标系正前方），左臂1坐标系K1_l为B*trans(y，-d_)*rot（x，45）=K1_l,右臂1坐标系K1_r为B*trans(y，d_)*rot（x，-45）=K1_r
        guodu_ << 1, 0, 0, 0, 0, cos(-M_PI / 4), -sin(-M_PI / 4), 0.232, 0, sin(-M_PI / 4), cos(-M_PI / 4), 0, 0, 0, 0, 1; // 过渡矩阵为trans(y，-d_)*rot（x，45）d_=0.2547 为机器人身体坐标系与左肩关节坐标系在y轴的距离
        f_a_guodu_ << 0, 0, 1, -1, 0, 0, 0, -1, 0;                                                                         // 末端坐标系到力传感器之间的姿态过渡矩阵
        local_attitude.setIdentity();
        local_position.setZero(4);

        // 机械臂参数调节
        cobra_Vel.resize(N2);
        cobra_Vel << 167, 167, 167, 167, 167, 167, 167; // 各底层电机基础速度,单位：每分钟多少转  具体到机械臂角度计算： cobra_Vel/60 * 360/Motor_Reduction_Ratio = 6 * cobra_Vel/Motor_Reduction_Ratio 度/秒
        Motor_Reduction_Ratio.resize(N2);
        Motor_Reduction_Ratio << 36, 36, 36, 36, 36, 36, 36; // 各电机减速比

        temperature = VectorXd::Zero(N2); // 电机的温度
        DianLiu = VectorXd::Zero(N2);     // 电机的电流

        secondSpeed = VectorXd::Zero(N2);  // 速度指令
        original_pos = VectorXd::Zero(N2); // 底层电机位置 单位：转
        pos = VectorXd::Zero(N2);          // 各电机位置信息 单位：°
        pos_init = VectorXd::Zero(N2);     // 初始化会自动录入启动时的初始位置
        pos_extend = VectorXd::Zero(N2);
        theta = VectorXd::Zero(N2);   // 末端控制状态电机角度。单位弧度。
        dtheta = VectorXd::Zero(N2);  // 末端控制状态解算的电机控制速度。单位弧度/s。
        end_move = VectorXd::Zero(6); // 末端运动的6个自由度

        pos_extend << 0, 0, -45, 90, 0, 0, 0; // 一键展开时的位置，重点在4号电机，方向不能反！！！！！
        Is_ChangeV.resize(N2);
        Is_ChangeV << 1, 1, 1, 1, 1, 1, 1; // 各电机是否受加减速影响，1为可以加减速，0为屏蔽加减速。
        DH_sign.resize(N2);
        DH_sign << -1, 1, -1, 1, -1, 1, -1; // 电机摆放对DH建模正方向的影响
        cobra_motor_sign.resize(N2);
        cobra_motor_sign << 1, -1, 1, 1, 1, 1, 1; // 电机与遥控器正反转适配，只与关节控制有关

        // DH参数输入
        DH_a.resize(N2);
        DH_alpha.resize(N2);
        DH_d.resize(N2);
        DH_bias.resize(N2); // theta的偏置
        DH_a << 0, 0, 0, 0, 0, 0, 0.057;
        DH_alpha << -90, -90, -90, 90, -90, 90, 0;
        // DH_d << 0, 0, 0.274, 0, 0.2555, 0, 0;
        // DH_d << 0, 0, 0.292, 0, 0.323, 0, 0; // 仿真环境下自带7Dof
        DH_d << 0, 0, 0.2738, 0, 0.2555, 0, 0; // 仿真环境下、实物下cc_arm
        DH_bias << 0, 90, 90, 0, 90, -90, 0;
        DH_alpha = DH_alpha.array() * M_PI / 180; // 弧度化,Eigen::Array类重载了+ ， - ，* ,/ 运算符，可以直接用这些运算符对Array对象进行操作。相乘操作是对应的数字相乘，相除是对应的元素相除。
        DH_bias = DH_bias.array() * M_PI / 180;

        // 闭环各参数输入
        FbEndmove = VectorXd::Zero(6);
        joint_vel = VectorXd::Zero(N2); // 各个电机反馈的转动速度 弧度/s

        secondend_pos_d.data.resize(6);
        secondend_pos_fb.data.resize(6);
        secondend_vel_d.data.resize(6);
        secondend_vel_fb.data.resize(6);
        RHS_trans.data.resize(9);
        secondend_FT.data.resize(6);
    }

private:
    ros::Subscriber joy_sub_;
    // ros::Subscriber control_sub_;
    ros::Subscriber secondCobraSubFromInnfos_;

    ros::Publisher cobra_angle_pub_;
    // ros::Publisher cobra_to_rviz_pub_;
    ros::Publisher secondCobraPubToInnfos_;
    ros::Publisher secondend_pos_d_;
    ros::Publisher secondend_pos_fb_;
    ros::Publisher secondend_vel_d_;
    ros::Publisher secondend_vel_fb_;
    ros::Publisher secondend_FT_;
    ros::Publisher RHS_trans_;

    ros::Timer controller;

    nubot_msgs::innfosSub secondCobraPubToInnfosMsg;
    // nubot_msgs::cobraToRviz cobraToRviz;
    nubot_msgs::angle_position timon_angle;
    // nubot_msgs::simSub secondCobraPubToInnfosMsg;
    std_msgs::Float64MultiArray secondend_pos_d;
    std_msgs::Float64MultiArray secondend_pos_fb;
    std_msgs::Float64MultiArray secondend_vel_d;
    std_msgs::Float64MultiArray secondend_vel_fb;
    std_msgs::Float64MultiArray secondend_FT;
    std_msgs::Float64MultiArray RHS_trans;

    // 机械臂参数调节
    int N1 = 7;
    int N2 = 7;          // CAN上电机个数，可设置为常量值const
    Matrix4d RHS;        // 用于计算末端位置
    Matrix4d guodu_;     // 支架基坐标系到1关节坐标系的全过渡矩阵
    Matrix3d f_a_guodu_; // 末端坐标系到力传感器的姿态过渡矩阵
    MatrixXd Jacob;

    VectorXd cobra_Vel;
    VectorXd Motor_Reduction_Ratio;
    std::vector<int> LimitMax = {100, 170, 20, 140, 100, 25, 60};       // 各电机最大限位 单位：° 度
    std::vector<int> LimitMin = {-100, -5, -160, -5, -100, -110, -105}; // 各电机最小限位 单位：° 度

    VectorXd temperature;
    VectorXd DianLiu;

    VectorXd secondSpeed;
    VectorXd original_pos;
    VectorXd pos;
    VectorXd pos_init;
    VectorXd pos_extend;
    VectorXd theta;
    VectorXd dtheta;
    VectorXd end_move;

    VectorXd Is_ChangeV;
    VectorXd DH_sign;

    // int DH_sign = { 1, -1, 1, 1, 1, -1, 1}; // 仿真环境下自带7Dof电机摆放对DH建模正方向的影响
    // int DH_sign = {1, 1, 1, -1, 1, 1, 1}; // 仿真环境下cc_arm电机摆放对DH建模正方向的影响
    VectorXd cobra_motor_sign;
    double dx = 0, dy = 0, dz = 0, rx = 0, ry = 0, rz = 0;

    int endV = 50;                 // 末端三个关节在一键模式的速度
    int cobra_Vel_Total_level = 3; // 机械臂末端运动速度档位数， 成倍增加，即3档是1档的3倍速，2档是1档的2倍速
    int end_mode_Total_level = 3;

    // DH参数输入
    VectorXd DH_a;
    VectorXd DH_alpha;
    VectorXd DH_d;
    VectorXd DH_bias; // theta的偏置

    // 导纳控制参数
    VectorXd joint_vel;
    VectorXd FbEndmove;
    VectorXd local_position; // 当前末端坐标位置 (列向量)
    Matrix3d local_attitude; // 当前末端姿态旋转矩阵

    // 各种标志位...
    double T_sample = 0.01; // 理想状态下指令发送到执行完毕的最大动作时间
    int left_or_right = 2;  // 左右臂控制切换，1为双臂，2为左臂，3为右臂
    bool now_control = true;
    bool cobra = true;
    bool flag_init = true;

    int NOWMODE = 0;
    float VMODE = 1;
    float PMODE = 2;
    bool enableOK = false;
    int need_v = 1;                 // 标志位，程序中不应改变
    int need_p = 2;                 // 标志位，
    double rotate_resolution = 0.2; // 末端旋转速度的分辨率（越大速度越快）默认值
    double speed_resolution = 0.05; // 末端平移速度的分辨率（越大速度越快）默认值

    float secondNeedMode = 0;
    float secondNowMode = 0;
    int flagX = 0, flagY = 0, flagA = 0, flagB = 0, flagAB = 0, flagAX = 0, flagXY = 0, flagBY = 0; // 八种位置模式组合键
    double flag[7] = {0};                                                                           // 普通遥控各电机遥控指令
    double flag2[6] = {0};                                                                          // 末端遥控各电机指令  dx , dy , dz, rx , ry , rz;

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
};

int main(int argc, char *argv[])
{
    /*******************************ROS*****************************/
    ros::init(argc, argv, "r7_jacob_joy_right");

    nubot_arm_node2 arm;

    ros::NodeHandle n;

    ros::spin();

    return 0;
}