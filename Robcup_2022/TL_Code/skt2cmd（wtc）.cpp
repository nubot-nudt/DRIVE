#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
// #include <syspes.h>
#include <sys/stat.h>
#include <string.h>
#include <arpa/inet.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <math.h>
#include <thread>
#include "dual_hand_serial/Hand_Ctrl.h"
#include "head_control/Head_Ctrl.h"
#include "nubot_msgs/innfosSub.h"

using namespace std;

int main(int argc, char **argv)
{
    ros::init(argc, argv, "skt2cmd");
    // 创建句柄（虽然后面没用到这个句柄，但如果不创建，运行时进程会出错）
    ros::NodeHandle n;

    ros::Publisher lh_pub = n.advertise<dual_hand_serial::Hand_Ctrl>("LHand_Command", 10);
    ros::Publisher rh_pub = n.advertise<dual_hand_serial::Hand_Ctrl>("RHand_Command", 10);

    dual_hand_serial::Hand_Ctrl rhand_command, lhand_command;

    ros::Rate loop_rate(60); // while以50Hz进行循环

    while (ros::ok())
    {

        // 计算关节角度
        // 1.指关节角度 存在问题：手指关节的映射角度可能需要标定一下，拇指的角度变化范围太小
        rhand_command.pinky = 180;
        rhand_command.ring = 180;
        rhand_command.middle = 180;
        rhand_command.index = 180;
        rhand_command.thumb = 180;
        lhand_command.pinky = 180;
        lhand_command.ring = 180;
        lhand_command.middle = 180;
        lhand_command.index = 180;
        lhand_command.thumb = 180;

        // 2.拇指关节角度
        rhand_command.thumb_joint = 90;
        lhand_command.thumb_joint = 90;
        lh_pub.publish(lhand_command);
        rh_pub.publish(rhand_command);

        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}
