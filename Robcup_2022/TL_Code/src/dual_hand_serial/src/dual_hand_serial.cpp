#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <string.h>
#include <stdio.h>
#include <chrono>
#include <dual_hand_serial/Hand_Ctrl.h>

#define rate 40 // 控制指令发送频率

using namespace std;

uint8_t rhand_cmd[18], lhand_cmd[18];
uint8_t hand_cmd_default[18] = {255, 255, 0x03, 0xA3, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 255, 255};
uint8_t mode; // 0 双手控制 1 右手控制 2 左手控制

// 右手控制指令回调函数
void RHand_Ctrl_Callback(const dual_hand_serial::Hand_Ctrl::ConstPtr &msg)
{
    rhand_cmd[4] = msg->pinky;
    rhand_cmd[6] = msg->ring;
    rhand_cmd[8] = msg->middle;
    rhand_cmd[10] = msg->index;
    rhand_cmd[12] = msg->thumb;
    rhand_cmd[14] = msg->thumb_joint;
}

// 左手控制指令回调函数
void LHand_Ctrl_Callback(const dual_hand_serial::Hand_Ctrl::ConstPtr &msg)
{
    lhand_cmd[4] = msg->pinky;
    lhand_cmd[6] = msg->ring;
    lhand_cmd[8] = msg->middle;
    lhand_cmd[10] = msg->index;
    lhand_cmd[12] = msg->thumb;
    lhand_cmd[14] = msg->thumb_joint;
}

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "dual_hand_serial");
    // 创建句柄
    ros::NodeHandle n;

    // 订阅右手控制指令
    ros::Subscriber r_sub = n.subscribe("/RHand_Command", 10, RHand_Ctrl_Callback);
    // 订阅左手控制指令
    ros::Subscriber l_sub = n.subscribe("/LHand_Command", 10, LHand_Ctrl_Callback);

    // 创建左右手串口serial类
    serial::Serial r_sp;
    serial::Serial l_sp;

    // 创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);

    // 设置要打开的串口名称
    r_sp.setPort("/dev/ttyUSB0");
    l_sp.setPort("/dev/ttyUSB1");

    // 设置串口通信的波特率
    r_sp.setBaudrate(115200);
    l_sp.setBaudrate(115200);

    // 串口设置timeout
    r_sp.setTimeout(to);
    l_sp.setTimeout(to);

    mode = 0; // 默认双手模式

    if (argc > 1)
    {
        if (strcmp(argv[1], "right"))
        {
            mode = 1;
        }
        else if (strcmp(argv[1], "left"))
        {
            mode = 2;
        }
    }

    if (mode == 0)
    {
        try
        {
            r_sp.open();
            l_sp.open();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port1 or port2.");
            return -1;
        }
        // 判断串口是否打开成功
        if (r_sp.isOpen() && l_sp.isOpen())
        {
            ROS_INFO_STREAM("/dev/ttyUSB0 is opened, right hand control activated.");
            ROS_INFO_STREAM("/dev/ttyUSB1 is opened, left hand control activated.");
        }
        else
        {
            return -1;
        }
    }
    if (mode == 1)
    {
        try
        {
            r_sp.open();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
            return -1;
        }
        // 判断串口是否打开成功
        if (r_sp.isOpen())
        {
            ROS_INFO_STREAM("/dev/ttyUSB0 is opened, right hand control activated.");
        }
        else
        {
            return -1;
        }
    }
    if (mode == 2)
    {
        try
        {
            l_sp.open();
        }
        catch (serial::IOException &e)
        {
            ROS_ERROR_STREAM("Unable to open port.");
            return -1;
        }
        // 判断串口是否打开成功
        if (l_sp.isOpen())
        {
            ROS_INFO_STREAM("/dev/ttyUSB1 is opened,left hand control activated.");
        }
        else
        {
            return -1;
        }
    }

    ros::Rate loop_rate(rate); // 数据包输出频率

    // 帧头
    rhand_cmd[0] = 0xFF;
    rhand_cmd[1] = 0xFF;
    // 上位机模式
    rhand_cmd[2] = 0x03;
    // 位置控制
    rhand_cmd[3] = 0xA3;
    // 帧尾
    rhand_cmd[16] = 0xFF;
    rhand_cmd[17] = 0xFF;

    // 帧头
    lhand_cmd[0] = 0xFF;
    lhand_cmd[1] = 0xFF;
    // 上位机模式
    lhand_cmd[2] = 0x03;
    // 位置控制
    lhand_cmd[3] = 0xA3;
    // 帧尾
    lhand_cmd[16] = 0xFF;
    lhand_cmd[17] = 0xFF;

    // 控制量的高字节部分，低字节在前，高字节在后，位置控制量范围0-90，不需要高字节，故将高字节置零
    // 控制量顺序分别 小指、无名指、中指、食指、拇指、拇指关节
    rhand_cmd[5] = 0;
    rhand_cmd[7] = 0;
    rhand_cmd[9] = 0;
    rhand_cmd[11] = 0;
    rhand_cmd[13] = 0;
    rhand_cmd[15] = 0;
    // 低字节初始值
    rhand_cmd[6] = 0;
    rhand_cmd[8] = 0;
    rhand_cmd[10] = 0;
    rhand_cmd[12] = 0;
    rhand_cmd[14] = 0;
    rhand_cmd[4] = 0;

    // 控制量的高字节部分，低字节在前，高字节在后，位置控制量范围0-90，不需要高字节，故将高字节置零
    // 控制量顺序分别 小指、无名指、中指、食指、拇指、拇指关节
    lhand_cmd[5] = 0;
    lhand_cmd[7] = 0;
    lhand_cmd[9] = 0;
    lhand_cmd[11] = 0;
    lhand_cmd[13] = 0;
    lhand_cmd[15] = 0;
    // 低字节初始值
    lhand_cmd[6] = 0;
    lhand_cmd[8] = 0;
    lhand_cmd[10] = 0;
    lhand_cmd[12] = 0;
    lhand_cmd[14] = 0;
    lhand_cmd[4] = 0;

    // 2s复位
    ROS_INFO("Reset Position");
    for (uint8_t i = 0; i < rate * 2; i++)
    {

        if (mode == 1)
            r_sp.write(hand_cmd_default, 18);
        if (mode == 2)
            l_sp.write(hand_cmd_default, 18);
        if (mode == 0)
        {
            r_sp.write(hand_cmd_default, 18);
            l_sp.write(hand_cmd_default, 18);
        }
        loop_rate.sleep();
    }

    ROS_INFO("Imitation Start");
    while (ros::ok())
    {

        ros::spinOnce();

        if (mode == 1)
        {
            r_sp.write(rhand_cmd, 18);
            ROS_INFO("%d,%d\n", rhand_cmd[4], rhand_cmd[6]);
        }
        if (mode == 2)
            l_sp.write(lhand_cmd, 18);
        if (mode == 0)
        {
            r_sp.write(rhand_cmd, 18);
            l_sp.write(lhand_cmd, 18);
        }

        loop_rate.sleep();
    }

    // 关闭串口
    if (mode == 1)
        r_sp.close();
    if (mode == 2)
        l_sp.close();
    if (mode == 0)
    {
        l_sp.close();
        r_sp.close();
    }

    return 0;
}
