#include <ros/ros.h>
#include <serial/serial.h>
#include <iostream>
#include <tf/tf.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <stdio.h>

#define g 9.8                  //重力加速度
#define d2r 3.1415927/180      //角度转弧度
#define rate 100               //imu数据包发送频率

double Acc_x, Acc_y, Yaw,W_z;         //imu原始数据 加速度 角度 角速度
double V_x, V_y, X,Y;                 //原始累计积分数据
double X_M, Y_M, Ax_0Err[rate/5], Ax_0Err_Aver,Vx_M,Pre_Ax,sum_temp;                     //
uint32_t sum = 0, count = 0;

tf::Quaternion Yaw_Q;
nav_msgs::Odometry odom;


int main(int argc, char** argv)
{
    ros::init(argc, argv, "serial_port");
    //创建句柄
    ros::NodeHandle n;

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("odom",1, true);
    static tf::TransformBroadcaster br,br2;
    static tf::Transform trans_w2b,trans_b2l;
    //创建一个serial类
    serial::Serial sp;
    //创建timeout
    serial::Timeout to = serial::Timeout::simpleTimeout(100);
    //设置要打开的串口名称
    sp.setPort("/dev/ttyUSB0");
    //设置串口通信的波特率
    sp.setBaudrate(115200);
    //串口设置timeout
    sp.setTimeout(to);

    try
    {
        //打开串口
        sp.open();
    }
    catch(serial::IOException& e)
    {
        ROS_ERROR_STREAM("Unable to open port.");
        return -1;
    }

    //判断串口是否打开成功
    if(sp.isOpen())
    {
        ROS_INFO_STREAM("/dev/ttyUSB0 is opened.");
    }
    else
    {
        return -1;
    }

    ros::Rate loop_rate(rate);			//数据包输出频率
    V_x = 0;
    V_y = 0;
    X = 0;
    Y = 0;

    while(ros::ok())
    {
        //获取缓冲区内的字节数
        size_t n = sp.available();
        //ROS_WARN("n= %d\n",n);
        if(n!=0)
        {
            uint8_t buffer[1024];
            uint8_t *p;
            p=buffer;

            //读出数据
            n = sp.read(buffer, n);
            for(int i=0; i<n; i++)
            {
//                //16进制的方式打印到屏幕
                //std::cout << std::hex << (buffer[i] & 0xff) << " ";

                if(*p == 0x55)
                {
                    switch(*(p+1))
                    {

                    case 0x51:                          //加速度
                        sum = *p+*(p+1)+*(p+2)+*(p+3)+*(p+4)+*(p+5)+*(p+6)+*(p+7)+*(p+8)+*(p+9);
                        sum = sum % 256;
                        if(sum == *(p+10))
                        {
                            Acc_x = (short(*(p+3)<<8 | *(p+2)))/32768.0*16*g;
                            Acc_y = (short(*(p+5)<<8 | *(p+4)))/32768.0*16*g;
                        }

                       // ROS_INFO("sum = %f",sum);
                        //Acc_z = (short(*(p+7)<<8 | *(p+6)))/32768.0*16*g;
                        break;
                    case 0x52:                          //角速度
                        sum = *p+*(p+1)+*(p+2)+*(p+3)+*(p+4)+*(p+5)+*(p+6)+*(p+7)+*(p+8)+*(p+9);
                        sum = sum % 256;
                        if(sum == *(p+10))
                        {
                            W_z = (short(*(p+7)<<8 | *(p+6)))/32768.0*180;
                            //ROS_INFO("7 = %0x 6 = %0x",*(p+7),*(p+6));
                        }

                        break;
                    case 0x53:                          //角度
                        sum = *p+*(p+1)+*(p+2)+*(p+3)+*(p+4)+*(p+5)+*(p+6)+*(p+7)+*(p+8)+*(p+9);
                        sum = sum % 256;
                        if(sum == *(p+10))
                        {
                            Yaw = (short(*(p+7)<<8 | *(p+6)))/32768.0*180;
                            //ROS_INFO("7 = %0x 6 = %0x",*(p+7),*(p+6));
                        }

                        break;
                    }

                }
                p++;
            }
            //std::cout << std::endl;
            //ROS_INFO("Acc_x=%f Acc_y=%f W_z=%f Yaw=%f",Acc_x,Acc_y,W_z,Yaw);

            //直接积分的速度和位移数据
            V_x = V_x + Acc_x / rate;
            X = X + V_x / rate;
            V_y = V_y + Acc_y/ rate;
            Y = Y + V_y / rate;

            //计算零漂
            if(abs(Pre_Ax-Acc_x) < 0.001)
            {
                count ++;
                Ax_0Err[count % (rate/5)] = Acc_x;
                sum_temp = 0;
                for(int i=0; i<(rate/5);i++)
                {
                    sum_temp = sum_temp + Ax_0Err[i];
                }
                Ax_0Err_Aver = sum_temp / (rate/5);

                Pre_Ax = Acc_x;
            }
            else
            {
                count = 0;
                for(int i=0; i<(rate/5);i++)
                {
                    Ax_0Err[i] = 0;
                }
                Pre_Ax = Acc_x;
            }

            //使用去除零漂之后的加速度积分得速度
            Vx_M = Vx_M + (Acc_x-Ax_0Err_Aver) / rate;

            X_M = X_M + Vx_M * cos(Yaw) / rate;
            Y_M = Y_M + Vx_M * sin(Yaw) / rate;

            Yaw_Q = tf::createQuaternionFromYaw(Yaw * d2r);
            //ROS_INFO("count=%d, Ax_0Err_Aver=%f",count,Ax_0Err_Aver);
//            ROS_INFO("V_x=%f V_y=%f",V_x,V_y);
            ROS_INFO("X=%f Y=%f",X,Y);
            ROS_INFO("X_M=%f Y_M=%f",X_M,Y_M);

            odom.header.stamp = ros::Time::now();
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            odom.pose.pose.position.x = X_M;
            odom.pose.pose.position.y = Y_M;
            odom.pose.pose.orientation.x = Yaw_Q.getX();
            odom.pose.pose.orientation.y = Yaw_Q.getY();
            odom.pose.pose.orientation.z = Yaw_Q.getZ();
            odom.pose.pose.orientation.w = Yaw_Q.getW();
            odom.twist.twist.linear.x = V_x;
            odom.twist.twist.angular.z = W_z;
            trans_w2b.setOrigin(tf::Vector3(X_M,Y_M,0));
            trans_w2b.setRotation(tf::Quaternion(Yaw_Q.x(),Yaw_Q.y(),Yaw_Q.z(),Yaw_Q.w()));
            trans_b2l.setOrigin(tf::Vector3(0,0,0));
            trans_b2l.setRotation(tf::Quaternion(0,0,0,1));
            std::cout << std::endl;
        }
        br.sendTransform(tf::StampedTransform(trans_w2b, ros::Time::now(), "odom","base_link"));
        //br2.sendTransform(tf::StampedTransform(trans_b2l, ros::Time::now(), "base_link","laser"));
        odom_pub.publish(odom);
        loop_rate.sleep();
    }

    //关闭串口
    sp.close();

    return 0;
}
