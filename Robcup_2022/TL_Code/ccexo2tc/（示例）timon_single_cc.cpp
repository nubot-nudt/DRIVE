#include <iostream>
#include <thread>
#include <cstdlib>
#include <chrono>
#include <cmath>

#include "innfos/actuatorcontroller.h"
#include <signal.h>
#include <string.h>
#include <iomanip>
#include <list>

#include <ros/ros.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
#include <std_msgs/Float64MultiArray.h>

#include "teleope_slave.h"
using   namespace   std;
using std::chrono::high_resolution_clock;
using std::chrono::duration;
using std::chrono::duration_cast;

    enum RafikiFlow
    {
        CLOSE=0,
        OPEN=1,
        WARMUP =2,
        WARE    =    3,
        PREPARE    =   4,
        INITIAL=5,
        TELEOPE=6,
        HOMO=7,
        HETERO=8,
        CLAMUP=9,
        PAUSE=10,
        STOP=11
    };

high_resolution_clock::time_point t1, t2, t3, t4;
double reconfigure_time;

int Teloope_modeR=1;
int Teloope_modeL=1;
bool exopause;

double ret;
bool run=1;
bool cf=true;
int modeFlag=0;
long int waitCount=0;
bool homeft=false;
int homeft_count=0;
int loop_count=0;
float T_sample=0.014;
int motorLreduction[7]={36,36,36,36,36,36,36};//初始化减速比
int motorRreduction[7]={36,36,36,36,36,36,36};

 void Clear(void)
    {
        t1 = high_resolution_clock::now();
        t2=t1;
    }

double GetValue(void)
    {
         t2 = high_resolution_clock::now();
        duration<double> time_span = duration_cast<duration<double>>(t2 - t1);
        ret=time_span.count();
        return ret;
    }   

    class nubot_dualArm_node :teleope_slave
    {
    private:

    ros::NodeHandle n;
    ActuatorController * pController;
    Actuator::ErrorsDefine ec;
    std::vector<ActuatorController::UnifiedID> uIDArray;    

    unsigned char manLactuator_id[7];
    unsigned char manRactuator_id[7];
    sensor_msgs::JointState motorL;
    sensor_msgs::JointState motorR;
    sensor_msgs::JointState jointL;
    sensor_msgs::JointState jointR;
    std_msgs::Int32 mode;


    geometry_msgs::Vector3 Current_slaveL_ShoulderPos;
    geometry_msgs::Vector3    Current_slaveL_ShoulderVel;
    sensor_msgs::JointState Current_slaveL_Joint;
    geometry_msgs::Vector3 Target_masterL_ShoulderVel;
    std_msgs::Float64MultiArray Target_masterL_OtherJointVel;

    geometry_msgs::Vector3 Current_slaveR_ShoulderPos;
    geometry_msgs::Vector3    Current_slaveR_ShoulderVel;
    sensor_msgs::JointState Current_slaveR_Joint;
    geometry_msgs::Vector3 Target_masterR_ShoulderVel;
    std_msgs::Float64MultiArray Target_masterR_OtherJointVel;

    geometry_msgs::Twist Current_slaveL_EndPos;    
    geometry_msgs::Twist Current_slaveR_EndPos;
    geometry_msgs::Twist Current_slaveL_EndVel;    
    geometry_msgs::Twist Current_slaveR_EndVel;
    geometry_msgs::Twist Target_masterL_EndVel;
    geometry_msgs::Twist Target_masterR_EndVel;


    ros::Subscriber slave_mode_sub;
    ros::Subscriber homoControllerSlaveL_ShoulderVel_sub ;
    ros::Subscriber homoControllerSlaveL_OtherJointVel_sub;
    ros::Subscriber homoControllerSlaveR_ShoulderVel_sub;
    ros::Subscriber homoControllerSlaveR_OtherJointVel_sub;

    ros::Subscriber heteroControllerSlaveL_EndVel_sub;
    ros::Subscriber heteroControllerSlaveR_EndVel_sub;

    ros::Publisher slaveL_ShoulderPos_pub;
    ros::Publisher slaveL_ShoulderVel_pub;
    ros::Publisher slaveL_Joint_pub;
    ros::Publisher slaveL_Motor_pub;
    ros::Publisher slaveR_ShoulderPos_pub;
    ros::Publisher slaveR_ShoulderVel_pub;
    ros::Publisher slaveR_Joint_pub;
    ros::Publisher slaveR_Motor_pub;

    ros::Publisher slaveL_EndPos_pub;
    ros::Publisher slaveR_EndPos_pub;
    ros::Publisher slaveL_EndVel_pub;
    ros::Publisher slaveR_EndVel_pub;


    double Target_slaveL_MotorVel[7];
    double Target_slaveR_MotorVel[7];
    double *motorLposition;
    double *motorRposition;
    double *jointLposition;
    double *jointRposition;
    double *motorLvelocity;
    double *motorRvelocity;
    double *jointLvelocity;
    double *jointRvelocity;



    public:
        nubot_dualArm_node (/* args */);
        ~nubot_dualArm_node ();
    void slaveModeCallback(const std_msgs::Int32::ConstPtr &teleopMode);
    void homoControllerSlaveL_ShoulderVelCallback(const geometry_msgs::Vector3::ConstPtr &ShoulderEulerVel);
    void homoControllerSlaveL_OtherJointVelCallback(const std_msgs::Float64MultiArray::ConstPtr &OtherJointVel);

    void homoControllerSlaveR_ShoulderVelCallback(const geometry_msgs::Vector3::ConstPtr &ShoulderEulerVel);
    void homoControllerSlaveR_OtherJointVelCallback(const std_msgs::Float64MultiArray::ConstPtr &OtherJointVel);

    void heteroControllerSlaveL_EndVelCallback(const geometry_msgs::Twist::ConstPtr &EndPoseVel);
    void heteroControllerSlaveR_EndVelCallback(const geometry_msgs::Twist::ConstPtr &EndPoseVel);
    
    double *motorPos2jointPos(double* motorPos);
    double *motorVel2jointVel(double *motorVel);
    double *jointPos2motorPos(double* jointPos);
    double *jointVel2motorVel(double *jointVel);  

    //控制器
    void homoSlaveL_Controller();
    void heteroSlaveLController();
    
    void homoSlaveR_Controller();
    void heteroSlaveR_Controller();

    void initialParam();
    void man2initialPos();
    void manFclosePos2initialPos();

    };
    
    nubot_dualArm_node ::nubot_dualArm_node (/* args */)
    {
        motorL.header.stamp=ros::Time::now();
        motorL.name={"motorL1","motorL2","motorL3","motorL4","motorL5","motorL6","motorL7"};motorL.position={0,0,0,0,0,0,0};motorL.velocity={0,0,0,0,0,0,0};motorL.effort={0,0,0,0,0,0,0};

        motorR.header.stamp=ros::Time::now();
        motorR.name={"motorR1","motorR2","motorR3","motorR4","motorR5","motorR6","motorR7"};motorR.position={0,0,0,0,0,0,0};motorR.velocity={0,0,0,0,0,0,0};motorR.effort={0,0,0,0,0,0,0};
        jointL.header.stamp=ros::Time::now();
        jointL.name={"jointL1","jointL2","jointL3","jointL4","jointL5","jointL6","jointL7"};jointL.position={0,0,0,0,0,0,0};jointL.velocity={0,0,0,0,0,0,0};jointL.effort={0,0,0,0,0,0,0};

        jointR.header.stamp=ros::Time::now();
        jointR.name={"jointR1","jointR2","jointR3","jointR4","jointR5","jointR6","jointR7"};jointR.position={0,0,0,0,0,0,0};jointR.velocity={0,0,0,0,0,0,0};jointR.effort={0,0,0,0,0,0,0};
        
        Target_masterL_OtherJointVel.data.resize(4);
        Target_masterR_OtherJointVel.data.resize(4);

        slave_mode_sub =n.subscribe<std_msgs::Int32>("/teleope/mode",1,boost::bind(&nubot_dualArm_node::slaveModeCallback,this,_1));
        homoControllerSlaveL_ShoulderVel_sub =n.subscribe<geometry_msgs::Vector3>("/HomoCtrL/shoulder/rot_vel",1,boost::bind(&nubot_dualArm_node::homoControllerSlaveL_ShoulderVelCallback,this,_1));
        homoControllerSlaveL_OtherJointVel_sub =n.subscribe<std_msgs::Float64MultiArray>("/HomoCtrL/otherJoint/vel",1,boost::bind(&nubot_dualArm_node::homoControllerSlaveL_OtherJointVelCallback,this,_1));
        homoControllerSlaveR_ShoulderVel_sub =n.subscribe<geometry_msgs::Vector3>("/HomoCtrR/shoulder/rot_vel",1,boost::bind(&nubot_dualArm_node::homoControllerSlaveR_ShoulderVelCallback,this,_1));
        homoControllerSlaveR_OtherJointVel_sub =n.subscribe<std_msgs::Float64MultiArray>("/HomoCtrR/otherJoint/vel",1,boost::bind(&nubot_dualArm_node::homoControllerSlaveR_OtherJointVelCallback,this,_1));

        heteroControllerSlaveL_EndVel_sub=n.subscribe<geometry_msgs::Twist>("HeteroCtrL/end/twist", 1, boost::bind(&nubot_dualArm_node::heteroControllerSlaveL_EndVelCallback,this,_1));
        heteroControllerSlaveR_EndVel_sub=n.subscribe<geometry_msgs::Twist>("HeteroCtrR/end/twist", 1, boost::bind(&nubot_dualArm_node::heteroControllerSlaveR_EndVelCallback,this,_1));
         
        slaveL_ShoulderPos_pub = n.advertise<geometry_msgs::Vector3>("/ManL/shoulder/pose", 1);
        slaveL_ShoulderVel_pub =n.advertise<geometry_msgs::Vector3>("/ManL/shoulder/rot_vel", 1);
        slaveL_Joint_pub =n.advertise<sensor_msgs::JointState>("/ManL/joint/state",1);
        slaveL_Motor_pub =n.advertise<sensor_msgs::JointState>("/ManL/motor/state",1);

        slaveR_ShoulderPos_pub =n.advertise<geometry_msgs::Vector3>("/ManR/shoulder/pose", 1);
        slaveR_ShoulderVel_pub =n.advertise<geometry_msgs::Vector3>("/ManR/shoulder/rot_vel", 1);;
        slaveR_Joint_pub =n.advertise<sensor_msgs::JointState>("/ManR/joint/state",1);;
        slaveR_Motor_pub =n.advertise<sensor_msgs::JointState>("/ManR/motor/state",1);;

        slaveL_EndPos_pub =n.advertise<geometry_msgs::Twist>("/ManL/end/pose", 1);
        slaveR_EndPos_pub =n.advertise<geometry_msgs::Twist>("/ManR/end/pose", 1);
        slaveL_EndVel_pub =n.advertise<geometry_msgs::Twist>("/ManL/end/twist", 1);
        slaveR_EndVel_pub =n.advertise<geometry_msgs::Twist>("/ManR/end/twist", 1);
       


    ActuatorController * pController = ActuatorController::initController();
    //ec Define an error type, ec==0x00 means no error, ec will be passed to pcontroller-> lookupActuators(ec) by reference,
    //when the error occurs, ec value will be modified by SDK to the corresponding error code
    Actuator::ErrorsDefine ec;
    //Find the connected actuators and return the UnifiedID of all actuators found.
    std::vector<ActuatorController::UnifiedID> uIDArray = pController->lookupActuators(ec);
    //If the size of the uIDArray is greater than zero, the connected actuators have been found

     if(uIDArray.size() > 0)
    {
	        if(pController->enableActuatorInBatch(uIDArray))
        {
            cout << "All actuators have been enabled successfully! " << endl;
        }
            /*********Initialize Manupulator Pose**********/
            ActuatorController::UnifiedID manLactuator_0 = uIDArray.at(0);
            ActuatorController::UnifiedID manLactuator_1 = uIDArray.at(1);
            ActuatorController::UnifiedID manLactuator_2 = uIDArray.at(2);
            ActuatorController::UnifiedID manLactuator_3 = uIDArray.at(3);
            ActuatorController::UnifiedID manLactuator_4 = uIDArray.at(4);
            ActuatorController::UnifiedID manLactuator_5 = uIDArray.at(5);
            ActuatorController::UnifiedID manLactuator_6 = uIDArray.at(6);

            ActuatorController::UnifiedID manRactuator_0 = uIDArray.at(7);
            ActuatorController::UnifiedID manRactuator_1 = uIDArray.at(8);
            ActuatorController::UnifiedID manRactuator_2 = uIDArray.at(9);
            ActuatorController::UnifiedID manRactuator_3 = uIDArray.at(10);
            ActuatorController::UnifiedID manRactuator_4 = uIDArray.at(11);
            ActuatorController::UnifiedID manRactuator_5 = uIDArray.at(12);
            ActuatorController::UnifiedID manRactuator_6 = uIDArray.at(13);

            manLactuator_id[0]=manLactuator_0.actuatorID;
            manLactuator_id[1]=manLactuator_1.actuatorID;
            manLactuator_id[2]=manLactuator_2.actuatorID;
            manLactuator_id[3]=manLactuator_3.actuatorID;
            manLactuator_id[4]=manLactuator_4.actuatorID;
            manLactuator_id[5]=manLactuator_5.actuatorID;
            manLactuator_id[6]=manLactuator_6.actuatorID;

            manRactuator_id[0]=manRactuator_0.actuatorID;
            manRactuator_id[1]=manRactuator_1.actuatorID;
            manRactuator_id[2]=manRactuator_2.actuatorID;
            manRactuator_id[3]=manRactuator_3.actuatorID;
            manRactuator_id[4]=manRactuator_4.actuatorID;
            manRactuator_id[5]=manRactuator_5.actuatorID;
            manRactuator_id[6]=manRactuator_6.actuatorID;

            for (int i = 0; i < 7; i++)
            {
                // pController->activateActuatorMode(manLactuator_id[i],Actuator::Mode_Profile_Pos);
                // pController->activateActuatorMode(manRactuator_id[i],Actuator::Mode_Profile_Pos);

                motorR.position[i]=pController->getPosition(manRactuator_id[i],true)/motorRreduction[i]*360;
                motorL.position[i]=pController->getPosition(manLactuator_id[i],true)/motorLreduction[i]*360;
                cout << "MAN Position of motor:  "<< "R_"  << i + 1 << ": " << motorR.position[i] <<" | "
                                                                                       << "L_"  << i + 1 << ": " << motorL.position[i]<< endl;
            }
            cin.get();
            mode.data=OPEN;
            while (ros::ok)
            {
            //准备阶段
            //ros::Duration timeout(10);
            //ros::topic::waitForMessage<std_msgs::Int32>("/teleope/mode");
            if((mode.data==WARMUP)&(modeFlag!=WARMUP))
            {
                for (int i = 0; i < 7; i++)
                    {
                    //    pController->activateActuatorMode(manLactuator_id[i],Actuator::Mode_Profile_Pos);
                        pController->activateActuatorMode(manRactuator_id[i],Actuator::Mode_Profile_Pos);
                    //    pController->setPosition(manLactuator_id[i],0);
                        pController->setPosition(manRactuator_id[i],0);
                    }
                        cout << "WARMUP" << endl;
                        modeFlag=WARMUP;
            }
            //ros::topic::waitForMessage<std_msgs::Int32>("/teleope/mode");
//motor[1]与joint[1]相反
             if((mode.data==WARE)&(modeFlag!=WARE))
            {
                    //pController->setPosition(manLactuator_id[3],6);//手臂垂直放下
                    pController->setPosition(manRactuator_id[3],-6);//手臂垂直放下
                    modeFlag=WARE;
            }
            //ros::topic::waitForMessage<std_msgs::Int32>("/teleope/mode");
            if((mode.data==PREPARE)&(modeFlag!=PREPARE))
            {
                for (int i = 0; i < 7; i++)
                    {
                        pController->setPosition(manRactuator_id[i],0);
//                        pController->setPosition(manLactuator_id[i],0);
                    }
                sleep(5);
                for (int i = 0; i < 7; i++)
                    {
//                        motorL.position[i]=pController->getPosition(manLactuator_id[i],true);
                        motorR.position[i]=pController->getPosition(manRactuator_id[i],true)/motorRreduction[i]*360;
                        cout << "MAN Position of motor:  "<< "R_"  << i + 1 << ": " << motorR.position[i] <<" | "
//                                                                                            << "L_"  << i + 1 << ": " << motorL.position[i]
                                                                                            << endl;
                    }
                        cout << "PREPARE" << endl;
                        modeFlag=PREPARE;
            }
            //ros::topic::waitForMessage<std_msgs::Int32>("/teleope/mode");
            if ((mode.data==TELEOPE)&(modeFlag!=TELEOPE))
            {
                    for (int i = 0; i < 7; i++)
                {
 //               pController->activateActuatorMode(manLactuator_id[i],Actuator::Mode_Vel);
                pController->activateActuatorMode(manRactuator_id[i],Actuator::Mode_Profile_Vel);    
                }
                    for (int i = 0; i < 7; i++)
                    {
//                        pController->setVelocity(manLactuator_id[i],0);
                        pController->setVelocity(manRactuator_id[i],0);
                    }
                    modeFlag=TELEOPE;
            }
                // ros::topic::waitForMessage<std_msgs::Int32>("/teleope/mode");
            if ((mode.data==HOMO)|(mode.data==HETERO))
                {
                    for (int i = 0; i < 7; i++)
                    {           
//                    pController->activateActuatorMode(manLactuator_id[i],Actuator::Mode_Profile_Vel);
                    //pController->activateActuatorMode(manRactuator_id[i],Actuator::Mode_Profile_Vel);
                    }
                    cout << "TeleopeMode:" <<  mode.data << endl;

                    while(cf)
                    {  
                        if((mode.data==CLAMUP)|(mode.data==PAUSE)|(mode.data==STOP))
                    {
                            for (int i = 0; i < 7; i++)
                            {
//                            motorL.position[i]=pController->getPosition(manLactuator_id[i],true);
                            motorR.position[i]=pController->getPosition(manRactuator_id[i],true)/motorRreduction[i]*360;
//                            pController->activateActuatorMode(manLactuator_id[i],Actuator::Mode_Profile_Pos);
                            pController->activateActuatorMode(manRactuator_id[i],Actuator::Mode_Profile_Pos);
//                            pController->setPosition(manLactuator_id[i],motorL.position[i]);
                            pController->setPosition(manRactuator_id[i],motorR.position[i]*motorRreduction[i]/360);
                            }
                            cout << "TeleopeMode:" <<  mode.data << endl;
                        break;
                    }
                        else
                    {
                        waitCount++;
                        if (GetValue()>=T_sample)//800Hz
                        {
                            cout << "loop time: " << setprecision(5) << GetValue() << " | T_sample: " << setprecision(5) << T_sample << endl;
                            Clear();
                            cout << "开始记时" << setprecision(5) << GetValue()  << endl;
                            for (int i = 0; i < 7; i++)
                            {
                           motorL.position[i]=pController->getPosition(manLactuator_id[i],true)/motorLreduction[i]*360;
                           motorR.position[i]=pController->getPosition(manRactuator_id[i],true)/motorRreduction[i]*360;
                            motorL.velocity[i]=pController->getVelocity(manLactuator_id[i],true)/motorLreduction[i]*360/60;
                            motorR.velocity[i]=pController->getVelocity(manRactuator_id[i],true)/motorRreduction[i]*360/60;
                            }
                            cout << "获取速度位置用时" << setprecision(5) << GetValue()  << endl;
                            //传递左右臂肩膀的姿态与旋转速度
                            
                            slaveL_Motor_pub.publish(motorL);
                            slaveR_Motor_pub.publish(motorR);  
                            
                            Vector3d slaveLshoulderPos; 
                            Vector3d slaveRshoulderPos; 
                            Vector3d slaveLshoulderVel; 
                            Vector3d slaveRshoulderVel; 
                            
                            motorRposition=&(motorR.position[0]);
                            motorLposition=&(motorL.position[0]);
                            motorRvelocity=&(motorR.velocity[0]);
                            motorLvelocity=&(motorL.velocity[0]);
                            
                            jointLposition=motorPos2jointPos(motorLposition);
                            jointLvelocity=motorVel2jointVel(motorLvelocity);
                            for (int i = 0; i < 7; i++)
                            {
                                jointL.position[i]=jointLposition[i];
                                jointL.velocity[i]=jointLvelocity[i];
                            }
                            slaveLshoulderPos = slave_shoulder_pos_FK(1,jointLposition[0],jointLposition[1],jointLposition[2]);

                            jointRposition=motorPos2jointPos(motorRposition);
                            jointRvelocity=motorVel2jointVel(motorRvelocity);
                            for (int i = 0; i < 7; i++)
                            {
                                jointR.position[i]=jointRposition[i];
                                jointR.velocity[i]=jointRvelocity[i];
                            }
                            slaveRshoulderPos = slave_shoulder_pos_FK(0,jointRposition[0],jointRposition[1],jointRposition[2]);
                            
                            slaveRshoulderVel = slave_shoulder_vel_FK(0,jointRposition[0],jointRposition[1],jointRposition[2],jointRvelocity[0],jointRvelocity[1],jointRvelocity[2]);
                            slaveLshoulderVel = slave_shoulder_vel_FK(1,jointLposition[0],jointLposition[1],jointLposition[2],jointRvelocity[0],jointRvelocity[1],jointRvelocity[2]);

                            cout << "肩部正运动学用时" << setprecision(5) << GetValue()  << endl; 
                            slaveL_Joint_pub.publish(jointL);
                            slaveR_Joint_pub.publish(jointR);  
                             
                            Current_slaveL_ShoulderPos.x=slaveLshoulderPos[0];
                            Current_slaveL_ShoulderPos.y=slaveLshoulderPos[1];
                            Current_slaveL_ShoulderPos.z=slaveLshoulderPos[2];
                            Current_slaveR_ShoulderPos.x=slaveRshoulderPos[0];
                            Current_slaveR_ShoulderPos.y=slaveRshoulderPos[1];
                            Current_slaveR_ShoulderPos.z=slaveRshoulderPos[2];
                            slaveL_ShoulderPos_pub.publish(Current_slaveL_ShoulderPos);
                            slaveR_ShoulderPos_pub.publish(Current_slaveR_ShoulderPos);

                            Current_slaveL_ShoulderVel.x=slaveLshoulderVel[0];
                            Current_slaveL_ShoulderVel.y=slaveLshoulderVel[1];
                            Current_slaveL_ShoulderVel.z=slaveLshoulderVel[2];
                            Current_slaveR_ShoulderVel.x=slaveRshoulderVel[0];
                            Current_slaveR_ShoulderVel.y=slaveRshoulderVel[1];
                            Current_slaveR_ShoulderVel.z=slaveRshoulderVel[2];
                            slaveL_ShoulderVel_pub.publish(Current_slaveL_ShoulderVel);
                            slaveR_ShoulderVel_pub.publish(Current_slaveR_ShoulderVel);


                            

                            //传递左右臂末端的位姿与旋转速度
                            Matrix<double,6,1> slaveLendPos; 
                            Matrix<double,6,1> slaveRendPos; 
                            Matrix<double,6,1> slaveLendVel; 
                            Matrix<double,6,1> slaveRendVel; 
                            slaveLendPos=slave_7l1_pos_FK(1,jointLposition);
                            slaveRendPos=slave_7l1_pos_FK(0,jointRposition);

                            slaveLendVel=slave_7l1_vel_FK(1,jointLposition,jointLvelocity);
                            slaveRendVel=slave_7l1_vel_FK(0,jointRposition,jointRvelocity);
                            cout << "整臂正运动学用时" << setprecision(5) << GetValue()  << endl; 
                            Current_slaveL_EndPos.linear.x=slaveLendPos[0];
                            Current_slaveL_EndPos.linear.y=slaveLendPos[1];
                            Current_slaveL_EndPos.linear.z=slaveLendPos[2];
                            Current_slaveL_EndPos.angular.x=slaveLendPos[3];
                            Current_slaveL_EndPos.angular.y=slaveLendPos[4];
                            Current_slaveL_EndPos.angular.z=slaveLendPos[5];

                            Current_slaveR_EndPos.linear.x=slaveRendPos[0];
                            Current_slaveR_EndPos.linear.y=slaveRendPos[1];
                            Current_slaveR_EndPos.linear.z=slaveRendPos[2];
                            Current_slaveR_EndPos.angular.x=slaveRendPos[3];
                            Current_slaveR_EndPos.angular.y=slaveRendPos[4];
                            Current_slaveR_EndPos.angular.z=slaveRendPos[5];

                            slaveL_EndPos_pub.publish(Current_slaveL_EndPos);
                            slaveR_EndPos_pub.publish(Current_slaveR_EndPos);

                            Current_slaveL_EndVel.linear.x=slaveLendVel[0];
                            Current_slaveL_EndVel.linear.y=slaveLendVel[1];
                            Current_slaveL_EndVel.linear.z=slaveLendVel[2];
                            Current_slaveL_EndVel.angular.x=slaveLendVel[3];
                            Current_slaveL_EndVel.angular.y=slaveLendVel[4];
                            Current_slaveL_EndVel.angular.z=slaveLendVel[5];

                            Current_slaveR_EndVel.linear.x=slaveRendVel[0];
                            Current_slaveR_EndVel.linear.y=slaveRendVel[1];
                            Current_slaveR_EndVel.linear.z=slaveRendVel[2];
                            Current_slaveR_EndVel.angular.x=slaveRendVel[3];
                            Current_slaveR_EndVel.angular.y=slaveRendVel[4];
                            Current_slaveR_EndVel.angular.z=slaveRendVel[5];

                            slaveL_EndVel_pub.publish(Current_slaveL_EndVel);
                            slaveR_EndVel_pub.publish(Current_slaveR_EndVel);

                            //计算IK
                            
//                         for (int i = 0; i < 7; i++)
//                     { 
//                         pController->setVelocity(manLactuator_id[i],Target_slaveL_MotorVel[i]);
//                         pController->setVelocity(manRactuator_id[i],Target_slaveR_MotorVel[i]);
//                     }
                        pController->setVelocity(manRactuator_id[0],Target_slaveR_MotorVel[0]*motorRreduction[0]/360*60);
                        pController->setVelocity(manRactuator_id[1],Target_slaveR_MotorVel[1]*motorRreduction[1]/360*60);
                        pController->setVelocity(manRactuator_id[2],Target_slaveR_MotorVel[2]*motorRreduction[2]/360*60);
                        cout << "Target_slaveR_MotorVel[0]*motorRreduction[0]/360*60: " << Target_slaveR_MotorVel[0]*motorRreduction[0]/360*60 << endl;
                        cout << "Target_slaveR_MotorVel[1]*motorRreduction[1]/360*60: " << Target_slaveR_MotorVel[1]*motorRreduction[1]/360*60 << endl;
                        cout << "Target_slaveR_MotorVel[2]*motorRreduction[2]/360*60: " << Target_slaveR_MotorVel[2]*motorRreduction[2]/360*60 << endl;

                        pController->setVelocity(manRactuator_id[3],Target_slaveR_MotorVel[3]*motorRreduction[3]/360*60);
                        // pController->setVelocity(manRactuator_id[4],Target_slaveR_MotorVel[4]*motorRreduction[4]/360*60);
                        // pController->setVelocity(manRactuator_id[5],Target_slaveR_MotorVel[5]*motorRreduction[5]/360*60);
                        // pController->setVelocity(manRactuator_id[6],Target_slaveR_MotorVel[6]*motorRreduction[6]/360*60);
                        pController->setVelocity(manRactuator_id[4],0);
                        pController->setVelocity(manRactuator_id[5],0);
                        pController->setVelocity(manRactuator_id[6],0);
                        cout << "Target_slaveR_MotorVel[3]*motorRreduction[3]/360*60: " << Target_slaveR_MotorVel[3]*motorLreduction[3]/360*60 << endl;
                        // cout << "Target_slaveR_MotorVel[4]*motorRreduction[4]/360*60: " << Target_slaveR_MotorVel[4]*motorLreduction[4]/360*60 << endl;
                        // cout << "Target_slaveR_MotorVel[5]*motorRreduction[5]/360*60: " << Target_slaveR_MotorVel[5]*motorLreduction[5]/360*60 << endl;
                        // cout << "Target_slaveR_MotorVel[6]*motorRreduction[6]/360*60: " << Target_slaveR_MotorVel[6]*motorRreduction[6]/360*60 << endl;
                        cout << "发送速度用时" << setprecision(5) << GetValue()  << endl; 
                        // pController->setVelocity(manRactuator_id[0],Target_slaveR_MotorVel[0]);
                        // pController->setVelocity(manRactuator_id[1],Target_slaveR_MotorVel[1]);
                        // pController->setVelocity(manRactuator_id[2],Target_slaveR_MotorVel[2]);
                        // pController->setVelocity(manRactuator_id[3],Target_slaveR_MotorVel[3]);
                        // pController->setVelocity(manRactuator_id[4],Target_slaveR_MotorVel[4]);
                        // pController->setVelocity(manRactuator_id[5],Target_slaveR_MotorVel[5]);
                        // pController->setVelocity(manRactuator_id[6],Target_slaveR_MotorVel[6]);
                        loop_count++;
                        for (int i = 0; i < 7; i++)
                        {
                            cout<< "MansMotor: "  << endl;
                            cout 
                            <<" |pos" << "R" << i << ": " << setw(7) << motorR.position[i] <<" L" << i << ": "  << setw(7) << motorL.position[i] 
                            << "|vel"  << "R" << i << ": " << setw(7) << motorR.velocity[i] << " L"  << i << ": "  << setw(7)<< motorL.velocity[i] 
                            << "|cur"  << "R" << i << ": " << setw(7) << motorR.effort[i]  << " L"  << i << ": " << setw(7)<< motorL.effort[i] 
                            << endl;  
                        }
                        cout << "【step】: " << loop_count << "   Void loop number: " << waitCount << "   caculation cost time: " << setprecision(5) << GetValue() << endl;
                        waitCount=0;   
                        }
                    }
                    ros::spinOnce();
                    }
                }              
            if (mode.data==CLOSE)
            {
                break;
            }
            cout << "等待操作" <<endl;     
            cout << "【当前模式】：" << mode.data << endl;   
            cout << "【当前状态】：" << endl;     
            for (int i = 0; i < 7; i++)
            {
                cout << "MAN Position of motor:  "<< "R_"  << i + 1 << ": " << motorR.position[i] <<" | "
                                                                                       << "L_"  << i + 1 << ": " << motorL.position[i]<< endl;
            }
            this_thread::sleep_for(std::chrono::milliseconds(1000));
            cout << "\033c";//清空屏幕
             ros::spinOnce();
            } 

            }
else
       cout << "未找到电机" << endl;
    }

    
    nubot_dualArm_node ::~nubot_dualArm_node ()
    {

//        pController->setPosition(manLactuator_id[4],0);//
        pController->setPosition(manRactuator_id[4],0);//
//        pController->setPosition(manLactuator_id[5],0);//
        pController->setPosition(manRactuator_id[5],0);//
//        pController->setPosition(manLactuator_id[6],0);//
        pController->setPosition(manRactuator_id[6],0);//

//        pController->setPosition(manLactuator_id[3],7);//
        pController->setPosition(manRactuator_id[3],-7);//
        ros::shutdown;
        std::cout << "Node closing！ " << std::endl;
        this_thread::sleep_for(std::chrono::milliseconds(3000));
        pController->disableAllActuators();

    }
    void nubot_dualArm_node::slaveModeCallback(const std_msgs::Int32::ConstPtr &teleopMode)
    {
        mode.data=teleopMode->data;
    //    cout << "成功接收模式指令" << endl;
    }
    void nubot_dualArm_node ::homoControllerSlaveL_ShoulderVelCallback(const geometry_msgs::Vector3::ConstPtr &ShoulderEulerVel)
    {  
        double ShoulderVel[3];
        double joint[3];
        Vector3d shouldeJointVel;
        double jointVel[7];
        double *targetVel;

        ShoulderVel[0]=Target_masterL_ShoulderVel.x=ShoulderEulerVel->x;
        ShoulderVel[1]=Target_masterL_ShoulderVel.y=ShoulderEulerVel->y;
        ShoulderVel[2]=Target_masterL_ShoulderVel.z=ShoulderEulerVel->z;

        joint[0]=jointLposition[0];
        joint[1]=jointLposition[1];
        joint[2]=jointLposition[2];

        shouldeJointVel=slave_shoulder_vel_IK(1,joint,ShoulderVel);
        jointVel[0]=shouldeJointVel[0];jointVel[1]=shouldeJointVel[1];jointVel[2]=shouldeJointVel[2];
        jointVel[3]=Target_masterL_OtherJointVel.data[0];
        jointVel[4]=Target_masterL_OtherJointVel.data[1];
        jointVel[5]=Target_masterL_OtherJointVel.data[2];
        jointVel[6]=Target_masterL_OtherJointVel.data[3];

       targetVel=jointVel2motorVel(jointVel);

        for (int i = 0; i <7; i++)
        {
            Target_slaveL_MotorVel[i]=targetVel[i];
        }
        cout << "成功接收肩部速度模式指令" << endl;
    }
    void nubot_dualArm_node ::homoControllerSlaveL_OtherJointVelCallback(const std_msgs::Float64MultiArray::ConstPtr &OtherJointVel)
    {
        Target_masterL_OtherJointVel.data=OtherJointVel->data;
    //    cout << "成功接收其他关节速度模式指令" << endl;
    }
    void nubot_dualArm_node ::homoControllerSlaveR_ShoulderVelCallback(const geometry_msgs::Vector3::ConstPtr &ShoulderEulerVel)
    {
        double ShoulderVel[3];
        double joint[3];
        Vector3d shouldeJointVel;
        double jointVel[7];
        double *targetVel;
        ShoulderVel[0]=Target_masterR_ShoulderVel.x=ShoulderEulerVel->x;
        ShoulderVel[1]=Target_masterR_ShoulderVel.y=ShoulderEulerVel->y;
        ShoulderVel[2]=Target_masterR_ShoulderVel.z=ShoulderEulerVel->z;
        cout << "  ShoulderVel[0]= " <<ShoulderVel[0] <<"  ShoulderVel[1]= " <<ShoulderVel[1] <<"  ShoulderVel[2]= " <<ShoulderVel[2] << endl;
        joint[0]=jointR.position[0];
        joint[1]=jointR.position[1];
        joint[2]=jointR.position[2];
        if ((abs(ShoulderVel[0])<0.01)&&(abs(ShoulderVel[1])<0.01)&&(abs(ShoulderVel[2])<0.01))
        shouldeJointVel<< 0,0,0;
        else
        shouldeJointVel=slave_shoulder_vel_IK(0,joint,ShoulderVel);
        
        jointVel[0]=shouldeJointVel[0];jointVel[1]=shouldeJointVel[1];jointVel[2]=shouldeJointVel[2];
        for ( int i = 0; i < 4; i++)
        {
            if(abs(Target_masterR_OtherJointVel.data[i])<0.01)
            jointVel[3+i]=0;
            else
            jointVel[3+i]=Target_masterR_OtherJointVel.data[i];
        }
        cout << "  jointVel[0]= " <<jointVel[0] <<"  jointVel[1]= " <<jointVel[1] <<"  jointVel[2]= " <<jointVel[2] << endl;
       targetVel=jointVel2motorVel(jointVel);
        for (int i = 0; i <7; i++)
        {
            Target_slaveR_MotorVel[i]=targetVel[i];
        }
    //    cout << "接收到了ShoulderVel数据" <<endl;

    }
    void nubot_dualArm_node ::homoControllerSlaveR_OtherJointVelCallback(const std_msgs::Float64MultiArray::ConstPtr &OtherJointVel)
    {
        Target_masterR_OtherJointVel.data=OtherJointVel->data;
    //    cout << "接收到了OtherJointVel数据" <<endl;
    }

    void nubot_dualArm_node ::heteroControllerSlaveL_EndVelCallback(const geometry_msgs::Twist::ConstPtr &EndPoseVel)
    {
        double EndVel[6];
        double jointVel[7];
        double *targetVel;
        Matrix<double,7,1> shouldeJointVel;
        EndVel[0]=Target_masterL_EndVel.linear.x=EndPoseVel->linear.x;
        EndVel[1]=Target_masterL_EndVel.linear.y=EndPoseVel->linear.y;
        EndVel[2]=Target_masterL_EndVel.linear.z=EndPoseVel->linear.z;
        EndVel[3]=Target_masterL_EndVel.angular.x=EndPoseVel->angular.x;
        EndVel[4]=Target_masterL_EndVel.angular.y=EndPoseVel->angular.y;
        EndVel[5]=Target_masterL_EndVel.angular.z=EndPoseVel->angular.z;
        shouldeJointVel=slave_7l1_vel_IK(1,jointLposition,EndVel);
        targetVel=jointVel2motorVel(jointVel);
        
        for (int i = 0; i <7; i++)
        {
            Target_slaveL_MotorVel[i]=targetVel[i];
        }
    }
    void nubot_dualArm_node ::heteroControllerSlaveR_EndVelCallback(const geometry_msgs::Twist::ConstPtr &EndPoseVel)
    {
        double EndVel[6];
        double jointVel[7];
        double *targetVel;
        Matrix<double,7,1> shouldeJointVel;
        EndVel[0]=Target_masterR_EndVel.linear.x=EndPoseVel->linear.x;
        EndVel[1]=Target_masterR_EndVel.linear.y=EndPoseVel->linear.y;
        EndVel[2]=Target_masterR_EndVel.linear.z=EndPoseVel->linear.z;
        EndVel[3]=Target_masterR_EndVel.angular.x=EndPoseVel->angular.x;
        EndVel[4]=Target_masterR_EndVel.angular.y=EndPoseVel->angular.y;
        EndVel[5]=Target_masterR_EndVel.angular.z=EndPoseVel->angular.z;

        shouldeJointVel=slave_7l1_vel_IK(0,jointRposition,EndVel);
        targetVel=jointVel2motorVel(jointVel);
        
        for (int i = 0; i <7; i++)
        {
            Target_slaveR_MotorVel[i]=targetVel[i];
        }
    }
    double *nubot_dualArm_node ::motorPos2jointPos(double* motorPos)
    {
        static double  jointPos[7];
        jointPos[0]=-motorPos[0]/180*pi;
        jointPos[1]=-motorPos[1]/180*pi;//motor与joint均相反
        jointPos[2]=-motorPos[2]/180*pi;
        jointPos[3]=-motorPos[3]/180*pi;
        jointPos[4]=-motorPos[4]/180*pi;
        jointPos[5]=-motorPos[5]/180*pi;
        jointPos[6]=-motorPos[6]/180*pi;
        return jointPos;
    }
    double *nubot_dualArm_node ::motorVel2jointVel(double *motorVel)
    {
        static double  jointVel[7];
        jointVel[0]=-motorVel[0]/180*pi;
        jointVel[1]=-motorVel[1]/180*pi;//motor与joint均相反
        jointVel[2]=-motorVel[2]/180*pi;
        jointVel[3]=-motorVel[3]/180*pi;
        jointVel[4]=-motorVel[4]/180*pi;
        jointVel[5]=-motorVel[5]/180*pi;
        jointVel[6]=-motorVel[6]/180*pi;
        return jointVel;
    }
    double *nubot_dualArm_node ::jointPos2motorPos(double* jointPos)
    {
        static double  motorPos[7];
        motorPos[0]=-jointPos[0]*180/pi;
        motorPos[1]=-jointPos[1]*180/pi;//motor与joint均相反
        motorPos[2]=-jointPos[2]*180/pi;
        motorPos[3]=-jointPos[3]*180/pi;
        motorPos[4]=-jointPos[4]*180/pi;
        motorPos[5]=-jointPos[5]*180/pi;
        motorPos[6]=-jointPos[6]*180/pi;
        return motorPos;
    }
    double *nubot_dualArm_node ::jointVel2motorVel(double *jointVel)  
    {
        static double  motorVel[7];
        motorVel[0]=-jointVel[0]*180/pi;
        motorVel[1]=-jointVel[1]*180/pi;//motor与joint均相反
        motorVel[2]=-jointVel[2]*180/pi;
        motorVel[3]=-jointVel[3]*180/pi;
        motorVel[4]=-jointVel[4]*180/pi;
        motorVel[5]=-jointVel[5]*180/pi;
        motorVel[6]=-jointVel[6]*180/pi;
        return motorVel;
    }
    int main(int argc, char *argv[])
    {
        ros::init(argc, argv, "timon_cc");

            //Initialize the controller
    nubot_dualArm_node timon_dualArm;

    //teleope_slave teleope_slave;

    return 0;
    }