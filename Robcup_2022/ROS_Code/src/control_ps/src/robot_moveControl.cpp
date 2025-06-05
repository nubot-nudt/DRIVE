#include <iostream>
#include <string.h>
#include <ros/ros.h>
#include "sensor_msgs/Joy.h"
#include "geometry_msgs/Twist.h"

class robot_moveControl
{
public:
    robot_moveControl()
    {
        ros::NodeHandle n;
        joy_sub_ = n.subscribe("/joy",1,&robot_moveControl::joyCallback,this);
        speed_pub_ = n.advertise<geometry_msgs::Twist>("/robot_speed",1);
    }
    ~robot_moveControl(){}
    void joyCallback(const sensor_msgs::Joy& joy)
    {
        geometry_msgs::Twist speed;
        speed.linear.x = joy.axes[1];
        speed.linear.y = 0;
        speed.linear.z = 0;
        speed.angular.x = 0;
        speed.angular.y = 0;
        speed.angular.z = -joy.axes[3];
        speed_pub_.publish(speed);
    }
private:
    ros::Subscriber joy_sub_;
    ros::Publisher speed_pub_;

};


int main(int argc, char  **argv)
{
    ros::init(argc,argv,"robot_moveControl");
    robot_moveControl moveControl;
    ros::spin();
    return 0;
}
