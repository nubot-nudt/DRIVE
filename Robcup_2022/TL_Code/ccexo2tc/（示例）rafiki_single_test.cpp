#include <iostream>
#include "actuatorController.h"
#include <time.h>
#include <sstream>
#include <string>
#include <cmath>
#include <unistd.h>
#include <cstdlib>
#include <thread>
#include <signal.h>
#include <chrono>
#include <vector>
#include <future>


#include "ftsensorPerceptron.h"


#include <chrono>
#include <ros/ros.h>
#include <geometry_msgs/Wrench.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/JointState.h>
//#include <std_msgs/Float32MultiArray.h>
#include <dynamic_reconfigure/server.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/Int32.h>
#include "nubot_rafiki/rafikiConfig.h"
#include "nubot_rafiki/joint.h"
#include "teleope_homo_hetero.h"


#include <cstdio>
#include <cstring>
#include <termios.h>
#include "keyEvent.h"

// #define HOMO 1
// #define HETERO 1
//typedef void (*FnPtr)();//用于多线程调用的

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
        STOP=11,
    };

Sensor sensorR[3];//得定义成全局变量才能在别的文件中使用
Sensor sensorL[3];

Actuator motorR[9];//定义电机对象
Actuator motorL[9];//定义电机对象

sensor_msgs::JointState motorRstate;//角度
sensor_msgs::JointState motorLstate;//


geometry_msgs::Wrench ftRmsg[3];
geometry_msgs::Wrench ftLmsg[3];

std_msgs::Int32 mode;

high_resolution_clock::time_point t1, t2, t3, t4;
double reconfigure_time;

INT Teloope_modeR=1;
INT Teloope_modeL=1;
bool exopause;

double ret;
bool run=1;
bool cf=true;
long int waitCount=0;
bool homeft=false;
int homeft_count=0;
int loop_count=0;
float T_sample=0.0125;
//double pi=3.1415926;

// void signalHandler(int signum) {
//   struct termios attr;
//   tcgetattr(STDIN_FILENO, &attr);
//   attr.c_lflag |= ICANON | ECHO;
// //   tcsetattr(STDIN_FILENO, TCSANOW, &attr);

//   exit(signum);
// }

int sgn(double n)
{
   if(n>0)return 1;
   else if(n<0) return -1;
   else return 0;
}

int ifstatic(double n)
{
   if(-1<n<1)return 1;
   
   else return 0;
}

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
double SmoothSwitch(double  x, double a,float b ,float c)//平滑函数
    {
        double y=a/pi*2*atan(x/b)+c;
        return y;
    }

void tuneParaCallback(nubot_rafiki::rafikiConfig &config, uint32_t level)
{
    t4=t3;
    t3 = high_resolution_clock::now();
     exopause=config.pause;
     Teloope_modeR=config.Mode_RightArm;
     Teloope_modeL=config.Mode_LeftArm;

     duration<double> reconfigure_span = duration_cast<duration<double>>(t3 - t4);
     reconfigure_time=reconfigure_span.count();
     cout << "change the parameters! " << "reconfigure time: " << reconfigure_time << endl;
}
void reconfigureThread()
{
    cout << "reconfigureThread ID: " << this_thread::get_id() << endl;
    ros::spin();
}

class nubot_exo_node  : keyEvent,teleope_homo_hetero
{
private:
    ros::NodeHandle n;
    ftsensorPerceptron *kw6ftsensor= new ftsensorPerceptron;
    ftsensorPerceptron *kw6ftsensor1= new ftsensorPerceptron;
    actuatorController *ti5actuator= new actuatorController;//定义CAN通信控制对象
    actuatorController *ti5actuator1= new actuatorController;//定义CAN通信控制对象
    actuatorController *ti5actuator2= new actuatorController;//定义CAN通信控制对象
    actuatorController *ti5actuator3= new actuatorController;//定义CAN通信控制对象
    
    std_msgs::Float64MultiArray Target_masterR_ctrCur;
    std_msgs::Float64MultiArray Target_masterL_ctrCur;

    std_msgs::Float32 Target_masterRjoint0_ctrVel;
    std_msgs::Float32 Target_masterLjoint0_ctrVel;
    std_msgs::Float32 ctrR_vel0;//
    std_msgs::Float32 ctrL_vel0;//

    std_msgs::Float32 ctr_cur[9];
    std_msgs::Float32 ctr_curL[9];

    sensor_msgs::JointState jointRstate;//exoRjoint弧度
    sensor_msgs::JointState jointLstate;//exoLjoint



    geometry_msgs::Vector3 masterR_ShoulderVel;
    geometry_msgs::Vector3 masterL_ShoulderVel;
    geometry_msgs::Vector3 masterR_ShoulderPos;
    geometry_msgs::Vector3 masterL_ShoulderPos;
    
    geometry_msgs::Twist masterR_EndVel;
    geometry_msgs::Twist masterL_EndVel;
    geometry_msgs::Twist masterR_EndPos;
    geometry_msgs::Twist masterL_EndPos;
    




		//Actuator scapula1;
    //  enum RafikiFlow
    // {
    //     CLOSE=0,
    //     OPEN=1,
    //     WARMUP =2,
    //     WARE    =    2,
    //     PREPARE    =   3,
    //     INITIAL=4,
    //     TELEOPE=5,
    //     PAUSE=6,
    //     CLAMUP=7,
    // };

        //double (*ftR)[6];
        //double (*ftL)[6];
        //过渡变量

        double ForceR[3][6];
        double ForceL[3][6];
        double Forceadd[3][6]={0};
        double ForceLadd[3][6]={0};
        double ftR_offset[3][6]={0};
        double ftL_offset[3][6]={0};
        //用于控制
        double ForceRx[3];//单位都是mN
        double ForceRy[3];
        double ForceRz[3];
        double Torquex[3];//单位都是mNm
        double TorqueRy[3];
        double TorqueRz[3];

        double ForceLx[3];
        double ForceLy[3];
        double ForceLz[3];
        double TorqueLx[3];
        double TorqueLy[3];
        double TorqueLz[3];
        double posR[9]={0};//弧度
        double posL[9]={0};//弧度

        //用于批量
        UINT exoRsensorID[3];
        UINT exoLsensorID[3];
        UINT exoRmotorID[3];
        UINT exoLmotorID[3];
        UINT exoRmotorID_curCtr[8];//一关节采用位置控制
        UINT exoLmotorID_curCtr[8];

    ros::Publisher FTsensorL_shoulder_pub = n.advertise<geometry_msgs::Wrench>("/FTsensorL/exoShoulder", 1);
    ros::Publisher FTsensorL_elbow_pub = n.advertise<geometry_msgs::Wrench>("/FTsensorL/exoElbow", 1);
    ros::Publisher FTsensorL_wrist_pub = n.advertise<geometry_msgs::Wrench>("/FTsensorL/exoWrist", 1);
    ros::Publisher FTsensorR_shoulder_pub = n.advertise<geometry_msgs::Wrench>("/FTsensorR/exoShoulder", 1);
    ros::Publisher FTsensorR_elbow_pub = n.advertise<geometry_msgs::Wrench>("/FTsensorR/exoElbow", 1);
    ros::Publisher FTsensorR_wrist_pub = n.advertise<geometry_msgs::Wrench>("/FTsensorR/exoWrist", 1);

    //ros::Publisher ti5_pub = n.advertise<nubot_rafiki::joint>("/Actuator/ti5can", 1);

    //ros::Publisher ctr_cur_pub = n.advertise<std_msgs::Float32MultiArray>("/Actuator/control_cur", 1);
    ros::Publisher Target_masterR_ctrCur_pub = n.advertise<std_msgs::Float64MultiArray>("/HomoCtrR/exoMotor/control_cur", 1);
    ros::Publisher Target_masterL_ctrCur_pub = n.advertise<std_msgs::Float64MultiArray>("/HomoCtrL/exoMotor/control_cur", 1);
    ros::Publisher Target_masterRjoint0_ctrVel_pub = n.advertise<std_msgs::Float32>("HomoCtrR/exoMotor0/control_vel", 1);
    ros::Publisher Target_masterLjoint0_ctrVel_pub = n.advertise<std_msgs::Float32>("HomoCtrL/exoMotor0/control_vel", 1);
    ros::Publisher mode_pub = n.advertise<std_msgs::Int32>("/teleope/mode", 1);
    ros::Publisher masterL_ShoulderPos_pub = n.advertise<geometry_msgs::Vector3>("/ExoL/shoulder/pose", 1);
    ros::Publisher masterR_ShoulderPos_pub = n.advertise<geometry_msgs::Vector3>("/ExoR/shoulder/pose", 1);
    ros::Publisher masterL_ShoulderVel_pub = n.advertise<geometry_msgs::Vector3>("/ExoL/shoulder/rot_vel", 1);
    ros::Publisher masterR_ShoulderVel_pub = n.advertise<geometry_msgs::Vector3>("/ExoR/shoulder/rot_vel", 1);  
    ros::Publisher masterL_Joint_pub = n.advertise<sensor_msgs::JointState>("/ExoL/joint/state",1);
    ros::Publisher masterR_Joint_pub = n.advertise<sensor_msgs::JointState>("/ExoR/joint/state",1);
    
    ros::Publisher  masterL_EndPos_pub = n.advertise<geometry_msgs::Twist>("/ExoL/end/pose", 1);
    ros::Publisher  masterR_EndPos_pub = n.advertise<geometry_msgs::Twist>("/ExoR/end/pose", 1);
    ros::Publisher masterL_EndVel_pub = n.advertise<geometry_msgs::Twist>("/ExoL/end/twist", 1);
    ros::Publisher masterR_EndVel_pub = n.advertise<geometry_msgs::Twist>("/ExoR/end/twist", 1);
    dynamic_reconfigure::Server<nubot_rafiki::rafikiConfig> server;
    dynamic_reconfigure::Server<nubot_rafiki::rafikiConfig>::CallbackType ix;
    
    bool key_pressed = false;//按键触发暂停
    struct termios oldt;//按键触发暂停
    char ch;//按键触发暂停
public:
    nubot_exo_node();
    ~nubot_exo_node();
    void initialParam();
    void exo2initialPos();
    void exoFplacePos2initialPos();
    void exoFclosePos2initialPos();
    void exoFinitialPos2closePos();
    void exoFinitialPos2comfortPos();
    void exoSelfCheck();
    void sensorErrorClear();
    void masterHomo();//1表示左臂1L，0表示右臂0R
    void slaveHomo(bool RorL);
    void masterHetero(bool RorL);
    void slaveHetero(bool RorL);
    void pause();
    void chooseMode();
    void teleope();
    void joint2motor();
    void motor2joint();
};

nubot_exo_node::nubot_exo_node(/* args */)
{
        jointRstate.header.stamp=ros::Time::now();
        jointRstate.name={"jointR1","jointR2","jointR3","jointR4","jointR5","jointR6","jointR7","jointR8","jointR9"};
        jointRstate.position={0,0,0,0,0,0,0,0,0};
        jointRstate.velocity={0,0,0,0,0,0,0,0,0};
        jointRstate.effort={0,0,0,0,0,0,0,0,0};

        jointLstate.header.stamp=ros::Time::now();
        jointLstate.name={"jointL1","jointL2","jointL3","jointL4","jointL5","jointL6","jointL7","jointL8","jointL9"};
        jointLstate.position={0,0,0,0,0,0,0,0,0};
        jointLstate.velocity={0,0,0,0,0,0,0,0,0};
        jointLstate.effort={0,0,0,0,0,0,0,0,0};

        motorRstate=jointRstate;
        motorLstate=jointLstate;
        motorRstate.name={"motorR1","motorR2","motorR3","motorR4","motorR5","motorR6","motorR7","motorR8","motorR9"};
        motorLstate.name={"motorL1","motorL2","motorL3","motorL4","motorL5","motorL6","motorL7","motorL8","motorL9"};

        Target_masterR_ctrCur.data={0,0,0,0,0,0,0,0,0};
        Target_masterL_ctrCur.data={0,0,0,0,0,0,0,0,0};
     
     mode.data=CLOSE;
     mode_pub.publish(mode);
    //ix=boost::bind(&tuneParaCallback, _1, _2);
        bool  initCan0=ti5actuator->initCan(0,1000);
        //bool  initCan1=ti5actuator->initCan(1,1000);
        initialParam();//初始化电机、传感器参数
        //if (initCan0&initCan1)//boundRate默认设为1000，不建议修改
        if (initCan0)//boundRate默认设为1000，不建议修改
	{   
        mode.data=OPEN;
        mode_pub.publish(mode);
		cout << "OPEN阶段：CAN初始化成功！开始自检" << endl;
        //自检程序
        exoSelfCheck();
        cout << "自检完成" << endl;
        cin.get();
        exo2initialPos();

        // cout << "按enter电机回0，外骨骼从关闭位置到初始位置" << endl;
        // cin.get();
        // exoFclosePos2initialPos();
        // cout << "按enter电机回0，外骨骼从放置位置到初始位置" << endl;
        // cin.get();
        // exoFplacePos2initialPos();

		cout << "WARMUP阶段：外骨骼位于初始位置，开始佩戴外骨骼请踩N踏板" << endl;
        mode.data=WARMUP;
        mode_pub.publish(mode);
        cin.get();

        ti5actuator->setPosition(motorR[4],-30);
        // ti5actuator->setPosition(motorL[4],30);
        this_thread::sleep_for(std::chrono::milliseconds(2000));
        ti5actuator->setPosition(motorR[5],60);
        // ti5actuator->setPosition(motorL[5],-40);

        mode.data=WARE;
        mode_pub.publish(mode);        
        this_thread::sleep_for(std::chrono::milliseconds(3000));
        cout << "WARE阶段：请佩戴外骨骼，若佩戴完毕请按键" << endl;   
        cin.get();

        ti5actuator->setPosition(motorR[5],0);
        // ti5actuator->setPosition(motorL[5],0);
        this_thread::sleep_for(std::chrono::milliseconds(2000));
        ti5actuator->setPosition(motorR[4],0);
        // ti5actuator->setPosition(motorL[4],0);

        mode.data=PREPARE;//接下来开始自检
        mode_pub.publish(mode);
        cout << "PREPARE阶段：开始传感器归零" << endl;   
        cout << "请将传感器静置，进行归0操作请按键" << endl;
        cin.get();

        for (int i = 0; i < 3; i++)
        {
            exoRsensorID[i]=sensorR[i].id;
            exoLsensorID[i]=sensorL[i].id;
        }
        for (int i = 0; i < 9; i++)
        {
            exoRmotorID[i]=motorR[i+1].id;
            exoLmotorID[i]=motorL[i+1].id;
        }

        UINT exoRmotorID_curCtr[8];//一关节采用速度控制
        UINT exoLmotorID_curCtr[8];
        for (int i = 0; i < 8; i++)
        {
            exoRmotorID_curCtr[i]=motorR[i+1].id;
            exoLmotorID_curCtr[i]=motorL[i+1].id;
        }
        //server.setCallback(ix);
        //thread recofigure_thread(reconfigureThread);
        mode.data=INITIAL;
        cout << "INITIAL阶段：传感器归零" << endl; 
        mode_pub.publish(mode); 
        sensorErrorClear();
        mode.data=TELEOPE;
        mode_pub.publish(mode); 
        cout << "TELEOPE阶段：传感器归零完毕，准备选择操作模式" << endl; 
        cout << "按m键进入异构模式，按b键进入同构模式，按其他键则准备退出" << endl;
        chooseMode();

        // tcgetattr(STDIN_FILENO, &oldt);
        // // 捕获 SIGINT 信号（Ctrl+C）
        // signal(SIGINT, signalHandler);

        teleope();


        mode.data=CLAMUP;
        mode_pub.publish(mode); 
        this_thread::sleep_for(std::chrono::milliseconds(5000));
        cout << "CLAMUP阶段：外骨骼回到初始位置，关闭外骨骼请按键" << endl; 
        cin.get();
        exoFinitialPos2closePos();
        mode.data=CLOSE;
        mode_pub.publish(mode); 
        cout << "CLOSE阶段：外骨骼回到close位置" << endl; 

    }
            else
    {
        cout << "CAN初始化失败！" << endl;
    }
	    cout << "执行完毕，系统关闭" << endl;
	    delete ti5actuator;
	    delete ti5actuator1;
	    delete ti5actuator2;
	    delete ti5actuator3;
        delete kw6ftsensor;	        
        delete kw6ftsensor1;	         
    }



nubot_exo_node::~nubot_exo_node(){
 pause();//外骨骼停住
cout << "程序结束" << endl;
delete ti5actuator;
delete kw6ftsensor;	             

 ros::shutdown();
}
void nubot_exo_node::initialParam(){
    sensorR[0].id=26;
    sensorR[0].channel=1;
    sensorR[0].debicIndex=0;
    sensorR[1].id=28;
    sensorR[1].channel=1;
    sensorR[1].debicIndex=0;
    sensorR[2].id=30;
    sensorR[2].channel=1;
    sensorR[2].debicIndex=0;

    sensorL[0].id=20;
    sensorL[0].channel=1;
    sensorL[0].debicIndex=1;    
    sensorL[1].id=22;
    sensorL[1].channel=1;
    sensorL[1].debicIndex=1;   
    sensorL[2].id=24;
    sensorL[2].channel=1;
    sensorL[2].debicIndex=1;   
    for (int i = 0; i < 3; i++)
    {
        sensorR[i].errornum=0;
        sensorL[i].errornum=0;
    }
    
           for (int i = 0; i < 9; i++)
        {
            motorR[i].ordernum=i;
            motorR[i].id=11+i;
            

            motorL[i].ordernum=i;
            motorL[i].id=1+i;

            motorR[i].errornum=0;
            motorL[i].errornum=0;

            
            if (i<6)
            {
            motorR[i].channel=0;
            motorL[i].channel=0;
            motorR[i].debicIndex=0;
            motorL[i].debicIndex=1;
            }
            else
            {
            motorR[i].channel=1;
            motorL[i].channel=1;
            motorR[i].debicIndex=0;
            motorL[i].debicIndex=1;
            }
        }
        
        motorR[0].reduction=101;//电机减速比
        motorR[1].reduction=51;
        motorR[2].reduction=51;
        motorR[3]. reduction=51;
        motorR[4].reduction=51;
        motorR[5].reduction=51;
        motorR[6]. reduction=60;
        motorR[7].reduction=51;
        motorR[8].reduction=51;

        motorL[0].reduction=101;//电机减速比
        motorL[1].reduction=51;
        motorL[2].reduction=51;
        motorL[3]. reduction=51;
        motorL[4].reduction=51;
        motorL[5].reduction=51;
        motorL[6].reduction=60;
        motorL[7].reduction=101;
        motorL[8].reduction=51;

        motorR[0].posOffset=-65.2941;//电机位置偏置-17.2094
        motorR[1].posOffset=30.02335;//27.02335
        motorR[2].posOffset=20.0072;//13.0072
        motorR[3].posOffset=-92.3700;//-59.3700
        motorR[4].posOffset=-40.4941;
        motorR[5].posOffset=-60.8685;
        motorR[6].posOffset=6.1881;
        motorR[7].posOffset=9.5476;//11.5476
        motorR[8].posOffset=5.8813;//-20.0813

        motorL[0].posOffset=-11.3279;//电机位置偏置-17.2094
        motorL[1].posOffset=-25.454;//-21.8216
        motorL[2].posOffset=18.1161;//13.0072
        motorL[3].posOffset=4.9035;//4.43837
        motorL[4].posOffset=-6.4730;
        motorL[5].posOffset=7.7290;
        motorL[6].posOffset=-50.4174;//-55.4174
        motorL[7].posOffset=-18;//-63.2747
        motorL[8].posOffset=-3.9277;//1.9277

        for (int i = 0; i < 3; i++)
        {
            exoRsensorID[i]=sensorR[i].id;
            exoLsensorID[i]=sensorL[i].id;
        }
        for (int i = 0; i < 8; i++)
        {
            exoRmotorID_curCtr[i]=motorR[i+1].id;
            exoLmotorID_curCtr[i]=motorL[i+1].id;
        } 
        if(!kw6ftsensor->startFT(sensorR[0]))//如果设备0读到了传感器怎，设备号都取反
        {
            cout << "设备号相反，传感器设备号为0" << endl;
            for (int i = 0; i < 9; i++)
            {
                motorR[i].debicIndex=!motorR[i].debicIndex;
                motorL[i].debicIndex=!motorL[i].debicIndex;
            }  
            for (int i = 0; i < 3; i++)
            {
                sensorR[i].debicIndex=!sensorR[i].debicIndex;
                sensorL[i].debicIndex=!sensorL[i].debicIndex;
            }  
        }
        else
        {
            cout << "设备号相同，传感器设备号为1" << endl;
        }
        
}
void nubot_exo_node::exo2initialPos(){
        for (int i = 0; i < 9 ; i++)
        {
            ti5actuator->setPosition(motorR[i],0);
//            ti5actuator->setPosition(motorL[i],0);
        }
}
void nubot_exo_node::exoFplacePos2initialPos(){
//            ti5actuator->setPosition(motorL[5],10);
            ti5actuator->setPosition(motorR[5],-10);
            this_thread::sleep_for(std::chrono::milliseconds(1000));
//            ti5actuator->setPosition(motorL[3],-20);
            ti5actuator->setPosition(motorR[3],20);
            this_thread::sleep_for(std::chrono::milliseconds(1000));

//            ti5actuator->setPosition(motorL[0],15);
//            ti5actuator->setPosition(motorL[2],5);
            ti5actuator->setPosition(motorR[0],-15);
            ti5actuator->setPosition(motorR[2],-5);

            ti5actuator->setPosition(motorR[1],0);
//            ti5actuator->setPosition(motorL[1],0);
            
//            ti5actuator->setPosition(motorL[6],0);
//            ti5actuator->setPosition(motorL[7],0);
//            ti5actuator->setPosition(motorL[8],0);
            ti5actuator->setPosition(motorR[6],0);
            ti5actuator->setPosition(motorR[7],0);
            ti5actuator->setPosition(motorR[8],0);
            this_thread::sleep_for(std::chrono::milliseconds(3000));
            cout << "请抽走盒子,抽走后按enter键到initial位置"<<endl;
            cin.get();
            for (int i = 0; i < 9; i++)
            {
//                ti5actuator->setPosition(motorL[i],0);
                ti5actuator->setPosition(motorR[i],0);
            }
            

}
void nubot_exo_node::exoFclosePos2initialPos(){
//            ti5actuator->setPosition(motorL[1],0);
//            ti5actuator->setPosition(motorL[5],10);

            ti5actuator->setPosition(motorR[1],0);
            ti5actuator->setPosition(motorR[5],-10);
        this_thread::sleep_for(std::chrono::milliseconds(5000));
//            ti5actuator->setPosition(motorL[4],20);
            ti5actuator->setPosition(motorR[4],-20);
        this_thread::sleep_for(std::chrono::milliseconds(4000));
//            ti5actuator->setPosition(motorL[6],0);
//            ti5actuator->setPosition(motorL[7],0);
//            ti5actuator->setPosition(motorL[8],0);
            ti5actuator->setPosition(motorR[6],0);
            ti5actuator->setPosition(motorR[7],0);
            ti5actuator->setPosition(motorR[8],0);

//            ti5actuator->setPosition(motorL[3],0);
            ti5actuator->setPosition(motorR[3],0);
        this_thread::sleep_for(std::chrono::milliseconds(3000));            
 //           ti5actuator->setPosition(motorL[0],0);
 //           ti5actuator->setPosition(motorL[2],0);

            ti5actuator->setPosition(motorR[0],0);
            ti5actuator->setPosition(motorR[2],0);
        this_thread::sleep_for(std::chrono::milliseconds(2000));
//            ti5actuator->setPosition(motorL[4],0);
            ti5actuator->setPosition(motorR[4],0);
//            ti5actuator->setPosition(motorL[5],0);
            ti5actuator->setPosition(motorR[5],0);

        this_thread::sleep_for(std::chrono::milliseconds(4000));//延时5s，chrono库可做实时控制，最高可精确到纳秒     	

        cout.setf(ios::showpos); //强制在正数前加+号
        cout.setf(ios::showpoint); //强制显示小数点后的无效0 
        for (int i = 0; i < 9; i++)
        {
            motorRstate.position[i]=ti5actuator->getPosition(motorR[i]);//ID,channel 
 //           motorLstate.position[i]=ti5actuator->getPosition(motorL[i]);//ID,channel 
        cout << "EXO Position of motor:  "<< "R_"  << i + 1 << ": " << motorRstate.position[i] <<" | "
                                                                             << "L_"  << i + 1 << ": " << motorLstate.position[i]<< endl;
        }
        cout.unsetf(ios::showpos); //取消正数前加+号 
        cout.unsetf(ios::showpoint); //取消显示小数点后的无效0 
}
void nubot_exo_node::exoFinitialPos2closePos(){
//            ti5actuator->setPosition(motorL[3],-90);
//            ti5actuator->setPosition(motorL[5],20);

            ti5actuator->setPosition(motorR[3],90);
            ti5actuator->setPosition(motorR[5],-20);
        this_thread::sleep_for(std::chrono::milliseconds(3000));
            // ti5actuator->setPosition(motorL[2],-90);
            // ti5actuator->setPosition(motorL[0],90);
            // ti5actuator->setPosition(motorL[6],-90);
            // ti5actuator->setPosition(motorL[7],0);
            // ti5actuator->setPosition(motorL[8],0);
            
            ti5actuator->setPosition(motorR[2],90);
            ti5actuator->setPosition(motorR[0],-90);
            ti5actuator->setPosition(motorR[6],90);
            ti5actuator->setPosition(motorR[7],0);
            ti5actuator->setPosition(motorR[8],0);

        this_thread::sleep_for(std::chrono::milliseconds(2000));
            // ti5actuator->setPosition(motorL[1],-90);
            // ti5actuator->setPosition(motorL[3],-120);

            ti5actuator->setPosition(motorR[1],90);
            ti5actuator->setPosition(motorR[3],120);

        this_thread::sleep_for(std::chrono::milliseconds(5000));//延时4s，chrono库可做实时控制，最高可精确到纳秒     	
//            ti5actuator->setPosition(motorL[5],0);
            ti5actuator->setPosition(motorR[5],0);
//            ti5actuator->setPosition(motorL[3],-100);
            ti5actuator->setPosition(motorR[3],100);
        this_thread::sleep_for(std::chrono::milliseconds(5000));
            for (size_t i = 0; i < 9; i++)//所有关节放松
            {
//                ti5actuator->setCurrent(motorL[i],0);
                ti5actuator->setCurrent(motorR[i],0);
            }
}
void nubot_exo_node::exoFinitialPos2comfortPos(){
//            ti5actuator->setPosition(motorL[3],-90);
//            ti5actuator->setPosition(motorL[5],20);

            ti5actuator->setPosition(motorR[3],90);
            ti5actuator->setPosition(motorR[5],-20);
        this_thread::sleep_for(std::chrono::milliseconds(2000));
//            ti5actuator->setPosition(motorL[2],90);
//            ti5actuator->setPosition(motorL[0],90);
       
            ti5actuator->setPosition(motorR[2],-90);
//            ti5actuator->setPosition(motorL[0],-90);

        this_thread::sleep_for(std::chrono::milliseconds(1000));
            // ti5actuator->setPosition(motorL[1],-90);
            // ti5actuator->setPosition(motorL[3],-120);
            // ti5actuator->setPosition(motorL[5],-30);

            ti5actuator->setPosition(motorR[1],90);
            ti5actuator->setPosition(motorR[3],120);
            ti5actuator->setPosition(motorR[5],30);
}

void nubot_exo_node::exoSelfCheck(){
        if((kw6ftsensor->startFT(sensorR[0]))&(kw6ftsensor->startFT(sensorR[1]))&(kw6ftsensor->startFT(sensorR[2])))
        cout<< "右臂传感器自检完成" <<endl;
        else
        {
        cout<< "右臂传感器未检测出数据" <<endl;
        cin.get();
        }

        // if((kw6ftsensor->startFT(sensorL[0]))&(kw6ftsensor->startFT(sensorL[1]))&(kw6ftsensor->startFT(sensorL[2])))
        // cout<< "左臂传感器自检完成" <<endl;
        // else
        // {
        // cout<< "左臂传感器未检测出数据" <<endl;
        // cin.get();
        // }
        


        this_thread::sleep_for(std::chrono::milliseconds(500));//延时0.5s，chrono库可做实时控制，最高可精确到纳秒

        double *FtR;
        double *FtL;
        for (int i = 0; i < 3; i++)
        {      FtR=kw6ftsensor->getFTdata(sensorR[i]);
 //               FtL=kw6ftsensor->getFTdata(sensorL[i]);        
                for (int j  = 0; j < 6; j++)
            {
            ForceR[i][j]=*(FtR+j);
            ForceL[i][j]=*(FtL+j);
            }
            ftRmsg[i].force.x=ForceR[i][0]*9.81;
            ftRmsg[i].force.y=ForceR[i][1]*9.81;
            ftRmsg[i].force.z=ForceR[i][2]*9.81;
            ftRmsg[i].torque.x=ForceR[i][3]*9.81;
            ftRmsg[i].torque.y=ForceR[i][4]*9.81;
            ftRmsg[i].torque.z=ForceR[i][5]*9.81;

            ftLmsg[i].force.x=ForceL[i][0]*9.81;
            ftLmsg[i].force.y=ForceL[i][1]*9.81;
            ftLmsg[i].force.z=ForceL[i][2]*9.81;
            ftLmsg[i].torque.x=ForceL[i][3]*9.81; 
            ftLmsg[i].torque.y=ForceL[i][4]*9.81;
            ftLmsg[i].torque.z=ForceL[i][5]*9.81;

            //printf("fx= %2f Kg,fy= %2f Kg,fz= %2f Kg,mx= %2f KgM,my= %2f KgM,mz= %2f KgM\n",ForceR[i][0],ForceR[i][1],ForceR[i][2],ForceR[i][3],ForceR[i][4],ForceR[i][5]);
            printf("EXO_R: Fx= %2f N,Fy= %2f N,Fz= %2f N,Tx= %2f NM,Ty= %2f NM,Tz= %2f NM\n", ftRmsg[i].force.x, ftRmsg[i].force.y, ftRmsg[i].force.z,ftRmsg[i].torque.x,ftRmsg[i].torque.y,ftRmsg[i].torque.z);

            //printf("fx= %2f Kg,fy= %2f Kg,fz= %2f Kg,mx= %2f KgM,my= %2f KgM,mz= %2f KgM\n",ForceL[i][0],ForceL[i][1],ForceL[i][2],ForceL[i][3],ForceL[i][4],ForceL[i][5]);
            //printf("EXO_L: Fx= %2f N,Fy= %2f N,Fz= %2f N,Tx= %2f NM,Ty= %2f NM,Tz= %2f NM\n", ftLmsg[i].force.x, ftLmsg[i].force.y, ftLmsg[i].force.z,ftLmsg[i].torque.x,ftLmsg[i].torque.y,ftLmsg[i].torque.z);
        }
        cout << "电机自检" << endl;   
        cout.setf(ios::showpos); //强制在正数前加+号
        cout.setf(ios::showpoint); //强制显示小数点后的无效0 
    for (int i = 0; i < 9; i++)
    {
        motorRstate.position[i]=ti5actuator->getPosition(motorR[i]);//ID,channel 
//        motorLstate.position[i]=ti5actuator->getPosition(motorL[i]);//ID,channel 
       cout << "EXO Position of motor:  "<< "R_"  << i + 1 << ": " << motorRstate.position[i] <<" | "
                                                                             << "L_"  << i + 1 << ": " << motorLstate.position[i]
                                                                            << endl;
    }
    cout.unsetf(ios::showpos); //取消正数前加+号 
    cout.unsetf(ios::showpoint); //取消显示小数点后的无效0 
}
void nubot_exo_node::sensorErrorClear(){
    while(cf)
        {
            waitCount++;
         if (GetValue()>=T_sample)//800Hz
            {
            cout << "loop time: " << setprecision(5) << GetValue() << " | T_sample: " << setprecision(5) << T_sample << endl;
            Clear();

                    // timer1=time(NULL);
                    // printf("The difference is: %f seconds",difftime(timer2,timer1));
                    // timer2=timer1;
            
            // future<double(*)[6]> exoft1= async(&ftsensorPerceptron::getFTdata_batch, &*kw6ftsensor, exoRsensorID, 0 );
            // future<double(*)[6]> exoft2 = async(&ftsensorPerceptron::getFTdata_batch, &*kw6ftsensor, exoLsensorID, 1 );
        //future<double(*)[6]> exoft1= async(launch::async,&ftsensorPerceptron::getFTdata_batch,kw6ftsensor, exoLsensorID, 0);
        //future<double(*)[6]> exoft2 = async(ftsensorPerceptron::getFTdata_batch, exoLsensorID, 1 );
            //  ftR=kw6ftsensor->getFTdata_batch(exoRsensorID, 0,3);
            //  ftL=kw6ftsensor->getFTdata_batch(exoLsensorID, 1,3);
            //ftR=exoft1.get();
            //ftL=exoft2.get();
             //	cout << res.get() << endl;
            // ft[i]=kw6ftsensor->getFTdata(sensorR[i].id,0);
            // ftL[i]=kw6ftsensor->getFTdata(sensorL[i].id,0);
            double *motorR_PosVelCur;   
            double *motorL_PosVelCur;  
        for (int i = 0; i < 9; i++)
        {          
            motorR_PosVelCur=ti5actuator->getCurVelPos(motorR[i]);
            this_thread::sleep_for(std::chrono::microseconds(400));
            motorRstate.position[i]=*(motorR_PosVelCur);
            motorRstate.velocity[i]=*(motorR_PosVelCur+1);
            motorRstate.effort[i]=*(motorR_PosVelCur+2);
        }
        for (int i = 0; i < 9; i++)
        {          
//            motorL_PosVelCur=ti5actuator->getCurVelPos(motorL[i]);
            this_thread::sleep_for(std::chrono::microseconds(400));
            motorLstate.position[i]=*(motorL_PosVelCur);
            motorLstate.velocity[i]=*(motorL_PosVelCur+1);
            motorLstate.effort[i]=*(motorL_PosVelCur+2);
        }
        for (int i = 0; i < 9; i++)
        {
            cout<< "ExosMotor: " 
            <<" |pos " << "R" << i << ": " << setw(10) << motorRstate.position[i] <<" L" << i << ": "  << setw(10) << motorLstate.position[i] 
            << "|vel "  << "R" << i << ": " << setw(10) << motorRstate.velocity[i] << " L"  << i << ": "  << setw(10)<< motorLstate.velocity[i] 
            << "|cur "  << "R" << i << ": " << setw(10) << motorRstate.effort[i]  << " L"  << i << ": " << setw(10)<< motorLstate.effort[i] 
            << endl;  
        }

        double *FtR;
        double *FtL;
        for (int i = 0; i < 3; i++)
        {      
            FtR=kw6ftsensor->getFTdata(sensorR[i]);
            for (int j  = 0; j < 6; j++)
            {
                ForceR[i][j]=*(FtR+j);
            }
            ftRmsg[i].force.x=ForceR[i][0]*9.81;
            ftRmsg[i].force.y=ForceR[i][1]*9.81;
            ftRmsg[i].force.z=ForceR[i][2]*9.81;
            ftRmsg[i].torque.x=ForceR[i][3]*9.81;
            ftRmsg[i].torque.y=ForceR[i][4]*9.81;
            ftRmsg[i].torque.z=ForceR[i][5]*9.81;                
            this_thread::sleep_for(std::chrono::microseconds(300));
            //FtL=kw6ftsensor->getFTdata(sensorL[i]);        
            for (int j  = 0; j < 6; j++)
            {
                ForceL[i][j]=*(FtL+j);
            }
            ftLmsg[i].force.x=ForceL[i][0]*9.81;
            ftLmsg[i].force.y=ForceL[i][1]*9.81;
            ftLmsg[i].force.z=ForceL[i][2]*9.81;
            ftLmsg[i].torque.x=ForceL[i][3]*9.81;
            ftLmsg[i].torque.y=ForceL[i][4]*9.81;
            ftLmsg[i].torque.z=ForceL[i][5]*9.81;
                    //printf("fx= %2f Kg,fy= %2f Kg,fz= %2f Kg,mx= %2f KgM,my= %2f KgM,mz= %2f KgM\n",ForceR[i][0],ForceR[i][1],ForceR[i][2],ForceR[i][3],ForceR[i][4],ForceR[i][5]);
                    printf("exoRsensor: Fx= %2f N,Fy= %2f N,Fz= %2f N,Tx= %2f NM,Ty= %2f NM,Tz= %2f NM\n", ftRmsg[i].force.x, ftRmsg[i].force.y, ftRmsg[i].force.z,ftRmsg[i].torque.x,ftRmsg[i].torque.y,ftRmsg[i].torque.z);
                    //printf("exoLsensor: Fx= %2f N,Fy= %2f N,Fz= %2f N,Tx= %2f NM,Ty= %2f NM,Tz= %2f NM\n", ftLmsg[i].force.x, ftLmsg[i].force.y, ftLmsg[i].force.z,ftLmsg[i].torque.x,ftLmsg[i].torque.y,ftLmsg[i].torque.z);
        }
        
                   
//*************************消除传感器误差*****************************//
                   
                   if(homeft_count<100)
                        {
                            homeft_count++;
                            for(int i=0;i<3;i++)
                            {
                                for (int j = 0; j < 6; j++)
                                {
                                    Forceadd[i][j]=Forceadd[i][j]+ForceR[i][j];
                                    ForceLadd[i][j]=ForceLadd[i][j]+ForceL[i][j];
                                }
                             printf("Forceadd[0]= %2f Kg,Forceadd[1]= %2f Kg,Forceadd[2]= %2f Kg,Forceadd[3]= %2f KgM,Forceadd[4]= %2f KgM,Forceadd[5]= %2f KgM\n", Forceadd[i][0], Forceadd[i][1], Forceadd[i][2], Forceadd[i][3], Forceadd[i][4], Forceadd[i][5]); 
                             printf("ForceLadd[0]= %2f Kg,ForceLadd[1]= %2f Kg,ForceLadd[2]= %2f Kg,ForceLadd[3]= %2f KgM,ForceLadd[4]= %2f KgM,ForceLadd[5]= %2f KgM\n", ForceLadd[i][0], ForceLadd[i][1], ForceLadd[i][2], ForceLadd[i][3], ForceLadd[i][4], ForceLadd[i][5]); 
                            }
                            cout << "homeft_count: " << homeft_count << endl;   
                       }

                            loop_count++;
                            cout << "【step】: " << loop_count << "   Void loop number: " << waitCount << "   caculation cost time: " << setprecision(5) << GetValue() << endl;
                            waitCount=0;   

                    if(!homeft&&homeft_count>99)
                        {
                            cout << "偏移量" << endl;   
                            for(int i=0;i<3;i++)
                            {   
                                for (int j = 0; j <6 ; j++)
                                {
                                ftR_offset[i][j]=Forceadd[i][j]/homeft_count;
                                ftL_offset[i][j]=ForceLadd[i][j]/homeft_count;
                                }
                                printf("ftR_offset: fx= %2f Kg,fy= %2f Kg,fz= %2f Kg,mx= %2f KgM,my= %2f KgM,mz= %2f KgM\n", ftR_offset[i][0], ftR_offset[i][1], ftR_offset[i][2], ftR_offset[i][3], ftR_offset[i][4], ftR_offset[i][5]);
                                printf("ftL_offset: fx= %2f Kg,fy= %2f Kg,fz= %2f Kg,mx= %2f KgM,my= %2f KgM,mz= %2f KgM\n", ftL_offset[i][0], ftL_offset[i][1], ftL_offset[i][2], ftL_offset[i][3], ftL_offset[i][4], ftL_offset[i][5]);
                            }

                                homeft=true;//偏置后可以控制
                                break;
                        }

            }
        
        }
}
void nubot_exo_node::masterHomo(){

        float a0=61.3; //高斯拟合函数
        float b0=163.5; 
        float c0=82.61;

        //master肩部正运动学
        thread threadHomoRctrCur([&](){
        Vector3d ShoulderPos(0,0,0);
        Vector3d ShoulderVel(0,0,0);
            thread threadRshoulderFK([&](){
                ShoulderPos = master_shoulder_pos_FK(0,jointRstate.position[0],jointRstate.position[1],jointRstate.position[2],jointRstate.position[3],jointRstate.position[4]);
                masterR_ShoulderPos.x=ShoulderPos[0];
                masterR_ShoulderPos.y=ShoulderPos[1];
                masterR_ShoulderPos.z=ShoulderPos[2];
                masterR_ShoulderPos_pub.publish(masterR_ShoulderPos);
                ShoulderVel = master_shoulder_vel_FK(0,jointRstate.position[0],jointRstate.position[1],jointRstate.position[2],jointRstate.position[3],jointRstate.position[4],
                                                                                                        jointRstate.velocity[0],jointRstate.velocity[1],jointRstate.velocity[2],jointRstate.velocity[3],jointRstate.velocity[4]);
                masterR_ShoulderVel.x=ShoulderVel[0];
                masterR_ShoulderVel.y=ShoulderVel[1];
                masterR_ShoulderVel.z=ShoulderVel[2];
                masterR_ShoulderVel_pub.publish(masterR_ShoulderVel);
        });

            //master肩部静力运动学
            double curR[9]={0};
            //肩
            double ss_a_cur1=SmoothSwitch(motorRstate.velocity[1],200,4,0);
            curR[1]=SmoothSwitch(ForceRz[0],1800-ss_a_cur1,500,0)   + cos(posR[2])*(/*0.345*/0-0.6*sin(posR[3]))*ForceRz[0]+ 1*cos(posR[2])*sin(posR[3])*TorqueRy[0]/0.75+0.1*sin(posR[2])*ForceRy[0]-0*motorRstate.velocity[1];//1.25->1    2*cur+4*velocity=-Force方向是反的
            
            double ss_a_cur2=SmoothSwitch(motorRstate.velocity[2],200,80,0);//4200  0.15  
            //curR[2]=SmoothSwitch(ForceRz[0],1400-ss_a_cur2,500,0)-(500+1800*cos(posR[3]))*cos(posR[2])+(1300+800*sin(posR[5]))*sin(posR[2])*cos(posR[3])+0.25*ForceRz[0]*cos(posR[3]) +0.18*TorqueRy[0]/0.1*cos(posR[3])-5*motorRstate.velocity[2]; //0.1->0.05  加上前臂腕部1600-4600
            curR[2]=SmoothSwitch(ForceRz[0],1200-ss_a_cur2,350,0)-(350+1600*cos(posR[3]))*cos(posR[2])+(1200+800*sin(posR[5]))*sin(posR[2])*cos(posR[3])+0.25*ForceRz[0]*cos(posR[3]) -0.05*TorqueRy[0]/0.1*cos(posR[3])-5*motorRstate.velocity[2]; //0.1->0.05  加上前臂腕部1600-4600
            
            double ss_a_cur3=SmoothSwitch(motorRstate.velocity[3],200,80,0);//1650 0.15
            curR[3]=SmoothSwitch(ForceRy[0],1200-ss_a_cur3,350,0)+1800*cos(posR[5])*cos(posR[3])*cos(posR[2])-(2300+800*sin(posR[5]))*sin(posR[3])*cos(posR[2])+0.25*ForceRy[0]-0.15*TorqueRz[0]/0.1 -5*motorRstate.velocity[3];//0.15->0.1 加上前臂腕部950-1950

            //肘
            double ss_a_cur4=SmoothSwitch(motorRstate.velocity[4],200,40,0);//0.04
            curR[4]=SmoothSwitch(ForceRz[1],800-ss_a_cur4,400,0)+400*cos(posR[4])*sin(posR[2])*cos(posR[5])-(1200*sin(posR[4])+300)*sin(posR[3])*cos(posR[2])+0.02*ForceRz[1]+0.035*TorqueRy[1]/0.1-3*motorRstate.velocity[4];//-1000*cos(motorRstate[4].position/180*pi)*sin(posR[2])*cos(posR[5])+0.35*ForceRz[1]+0.35*TorqueRy[1]/0.088 -80*motorRstate[4].velocity;//0.15->0.1

            double ss_a_cur5=SmoothSwitch(motorRstate.velocity[5],150,40,0);
            curR[5]=SmoothSwitch(TorqueRz[1],800+ss_a_cur5,400,0)-880*cos(posR[5])*cos(posR[2])+1150*sin(posR[3])*sin(posR[5])-0.047*ForceRy[1]+0.015*TorqueRz[1]/0.1-0.025*ForceRx[1] -3*motorRstate.velocity[5];//0.15->0.1

            //腕
            double ss_a_cur6=SmoothSwitch(motorRstate.velocity[6],100,40,0);
            curR[6]=SmoothSwitch(Torquex[2],350-ss_a_cur6,150,0)+300*cos(posR[6])*cos(posR[5])+0.19*Torquex[2]/0.1 -1*motorRstate.velocity[6];//0.15->0.1

            double ss_a_cur7=SmoothSwitch(motorRstate.velocity[7],100,20,0);
            curR[7]=SmoothSwitch(ForceRz[2],280-ss_a_cur7,92.5,0)-130*sin(posR[7])*sin(posR[6])-0.2*ForceRz[2]-0.22*TorqueRy[2]/0.1 -1*motorRstate.velocity[7];//0.15->0.1

            double ss_a_cur8=SmoothSwitch(motorRstate.velocity[8],30,10,0);
            curR[8]=-SmoothSwitch(ForceRy[2],80+ss_a_cur8,40,0)-50*(cos(posR[8])*cos(posR[6])+sin(posR[8])*sin(posR[7])*sin(posR[6]))-0.03*ForceRy[2] -1*motorRstate.velocity[8];//0.15->0.1
           //安全限位
            if(abs(curR[1])>5000)
            curR[1]=5000*curR[1]/abs(curR[1]);
            if(abs(curR[2])>10000)
            curR[2]=10000*curR[2]/abs(curR[2]);
            if(abs(curR[3])>10000)
            curR[3]=10000*curR[3]/abs(curR[3]);
            if(abs(curR[4])>4000)
            curR[4]=4000*curR[4]/abs(curR[4]);
            if(abs(curR[5])>3000)
            curR[5]=3000*curR[5]/abs(curR[5]);                
            if(abs(curR[6])>1500)
            curR[6]=1500*curR[6]/abs(curR[6]);
            if(abs(curR[7])>1000)
            curR[7]=1000*curR[7]/abs(curR[7]);
            if(abs(curR[8])>1000)
            curR[8]=1000*curR[8]/abs(curR[8]);

            //活动空间限位
            if ((motorRstate.position[5]>90)&&(curR[5]>0))
            {
                curR[5]=0;
            }
            if ((motorRstate.position[5]<-60)&&(curR[5]<0))
            {
                curR[5]=0;
            }
            

            for (int i = 1; i < 9; i++)
            {                              
                // ti5actuator->setCurrent(motorR[i].id,motorR[i].channel,curR[i]);
                Target_masterR_ctrCur.data[i]=curR[i];
            }
            ti5actuator->setCurrent(motorR[1],curR[1]);
            this_thread::sleep_for(std::chrono::microseconds(300));
            ti5actuator->setCurrent(motorR[2],curR[2]);
            this_thread::sleep_for(std::chrono::microseconds(300));
            ti5actuator->setCurrent(motorR[3],curR[3]);
            this_thread::sleep_for(std::chrono::microseconds(300));
            ti5actuator->setCurrent(motorR[4],curR[4]);
            this_thread::sleep_for(std::chrono::microseconds(300));
            ti5actuator->setCurrent(motorR[5],curR[5]);
            this_thread::sleep_for(std::chrono::microseconds(300));

            // ti5actuator->setCurrent(motorR[6],curR[6]);
            // this_thread::sleep_for(std::chrono::microseconds(300));
            // ti5actuator->setCurrent(motorR[7],curR[7]);
            // this_thread::sleep_for(std::chrono::microseconds(300));
            // ti5actuator->setCurrent(motorR[8],curR[8]);
            // this_thread::sleep_for(std::chrono::microseconds(300));


            cout<< " curR[1]: " << curR[1]<< " curR[2]: " << curR[2]
                    << " curR[3]: " << curR[3]<< " curR[4]: " << curR[4]  
                    << " curR[5]: " << curR[5] << " curR[6]: " << curR[6]  
                    << " curR[7]: " << curR[7] << " curR[8]: " << curR[8] << endl;
            Target_masterR_ctrCur_pub.publish(Target_masterR_ctrCur);

       
           //肩肱节律
        float AOER=-asin(cos(posR[2])*cos(posR[3])) ;
        float dAOER=((sin(posR[2])*cos(posR[3]))*motorRstate.velocity[2]+sin(posR[3])*cos(posR[2])*motorRstate.velocity[3])/sqrt(1-(cos(posR[2])*cos(posR[3])));
        double  vel_R0=-(-2*a0/c0*dAOER*exp(-pow((AOER-posR[0]-163.5)/82.61,2)))/(1+2*a0/c0);
       
        //ti5actuator->setVelocity(motorR[0].id,motorR[0].channel,vel_R0);
        Target_masterRjoint0_ctrVel.data=vel_R0;    
        Target_masterRjoint0_ctrVel_pub.publish(Target_masterRjoint0_ctrVel);
        threadRshoulderFK.join();
        });

        thread threadHomoLctrCur([&](){
            Vector3d ShoulderPos(0,0,0);
            Vector3d ShoulderVel(0,0,0);
            //master肩部正运动学
                thread threadLshoulderFK([&](){
                ShoulderPos = master_shoulder_pos_FK(1,jointLstate.position[0],jointLstate.position[1],jointLstate.position[2],jointLstate.position[3],jointLstate.position[4]);
                masterL_ShoulderPos.x=ShoulderPos[0];
                masterL_ShoulderPos.y=ShoulderPos[1];
                masterL_ShoulderPos.z=ShoulderPos[2];

                masterL_ShoulderPos_pub.publish(masterL_ShoulderPos);
                ShoulderVel = master_shoulder_vel_FK(1,jointLstate.position[0],jointLstate.position[1],jointLstate.position[2],jointLstate.position[3],jointLstate.position[4],
                                                                                                       jointLstate.velocity[0],jointLstate.velocity[1],jointLstate.velocity[2],jointLstate.velocity[3],jointLstate.velocity[4]);
                masterL_ShoulderVel.x=ShoulderVel[0];
                masterL_ShoulderVel.y=ShoulderVel[1];
                masterL_ShoulderVel.z=ShoulderVel[2];
                masterL_ShoulderVel_pub.publish(masterL_ShoulderVel);
            });

            //master肩部静力运动学
            double curL[9]={0};
            //肩[1]相反[2]相反[3]相同
            double ss_b_cur1=SmoothSwitch(motorLstate.velocity[1],200,4,0);
            curL[1]=-SmoothSwitch(ForceLz[0],1200-ss_b_cur1,300,0)  - cos(posL[2])*(0.345-0.25*sin(posL[3]))*ForceLz[0]- 1*cos(posL[2])*sin(posL[3])*TorqueLy[0]/0.75-20*motorLstate.velocity[1];//1.25->1    2*cur+4*velocity=-Force方向是反的
            
            double ss_b_cur2=SmoothSwitch(motorLstate.velocity[2],400,8,0);
            curL[2]=-SmoothSwitch(ForceLz[0],2000-ss_b_cur2,500,0)+4200*cos(posL[2])-0.15*ForceLz[0]*cos(posL[3]) +0.35*TorqueLy[0]/0.15*cos(posL[3])-20*motorLstate.velocity[2]; //0.1->0.05  加上前臂腕部1600-4600

            double ss_b_cur3=SmoothSwitch(motorLstate.velocity[3],200,4,0);
            curL[3]=SmoothSwitch(ForceLy[0],1400-ss_b_cur3,250,0)-1650*cos(posL[3])+0.15*ForceLy[0]-0.35*TorqueLz[0]/0.15 -20*motorLstate.velocity[3];//0.15->0.1 加上前臂腕部950-1950

            //肘[4]相反[5]相同
            double ss_b_cur4=SmoothSwitch(motorLstate.velocity[4],200,80,0);
            curL[4]=-SmoothSwitch(ForceLz[1],800-ss_b_cur4,600,0)-1000*cos(posL[4])*sin(posL[2])*cos(posL[5])-0.035*ForceLz[1]+0.008*TorqueLy[1]/0.088-8*motorLstate.velocity[4];//-1000*cos(motorLstate[4].position/180*pi)*sin(posR[2])*cos(posR[5])+0.35*ForceRz[1]+0.35*TorqueRy[1]/0.088 -80*motorLstate[4].velocity;//0.15->0.1

            double ss_b_cur5=SmoothSwitch(motorLstate.velocity[5],100,2,0);
            curL[5]=-SmoothSwitch(ForceLy[1],500+ss_b_cur5,250,0)+900*cos(posL[5])*cos(posL[2])-0.06*ForceLy[1]+0.025*TorqueLz[1]/0.088 -8*motorLstate.velocity[5];//0.15->0.1

            //腕 [6]相同[7]相反[8]相同
            double ss_b_cur6=SmoothSwitch(motorLstate.velocity[6],100,2,0);
            curL[6]=SmoothSwitch(TorqueLx[2],400-ss_b_cur6,100,0)-200*cos(posL[6])*cos(posL[5])+0.2*TorqueLx[2]/0.1 -5*motorLstate.velocity[6];//0.15->0.1

            double ss_b_cur7=SmoothSwitch(motorLstate.velocity[7],100,2,0);
            curL[7]=-SmoothSwitch(ForceLz[2],210-ss_b_cur7,52.5,0)+130*sin(posL[7])*sin(posL[6])+0.25*ForceLz[2]+0.1*TorqueLy[2]/0.1 -5*motorLstate.velocity[7];//0.15->0.1

            double ss_b_cur8=SmoothSwitch(motorLstate.velocity[8],50,1,0);
            curL[8]=-SmoothSwitch(ForceLy[2],160+ss_b_cur8,40,0)+80*(cos(posL[8])*cos(posL[6])-sin(posL[8])*sin(posL[7])*sin(posL[6]))-0.125*ForceLy[2] -5*motorLstate.velocity[8];//0.15->0.1
            for (int i = 0; i < 9; i++)
            {
                //ti5actuator->setCurrent(motorL[i].id,motorL[i].channel,curL[i]);
                Target_masterL_ctrCur.data[i]=curL[i];
            }
            Target_masterL_ctrCur_pub.publish(Target_masterL_ctrCur);
        
            //肩肱节律
            float AOEL=-asin(cos(posL[2])*cos(posL[3])) ;
            float dAOEL=((sin(posL[2])*cos(posL[3]))*motorLstate.velocity[2]+sin(posL[3])*cos(posL[2])*motorLstate.velocity[3])/sqrt(1-(cos(posL[2])*cos(posL[3])));
            double  vel_L0=(-2*a0/c0*dAOEL*exp(-pow((AOEL+posL[0]-163.5)/82.61,2)))/(1+2*a0/c0);
        
            //ti5actuator->setVelocity(motorL[0].id,motorL[0].channel,vel_L0);
            Target_masterLjoint0_ctrVel.data=vel_L0;
            Target_masterLjoint0_ctrVel_pub.publish(Target_masterLjoint0_ctrVel);
            threadLshoulderFK.join();
        });
        threadHomoRctrCur.join();
        threadHomoLctrCur.join();
        cout << "Actuator sample time: " << setprecision(5) << GetValue() << endl;
    // thread setCur[2];
    // setCur[0]=thread(&actuatorController::setCurrent_batch, &*ti5actuator,exoRmotorID_curCtr,0,cur,8);
    // setCur[1]=thread(&actuatorController::setCurrent_batch, &*ti5actuator,exoLmotorID_curCtr,1,curL,8);
    // setCur[0].join();
    // setCur[1].join();
}

void nubot_exo_node::masterHetero(bool RorL){
    //master逆运动学
    thread threadHeteroRctrVel([&](){
        double *jointposition;
        double *jointvelocity;
        double *ft3wrench;
        Matrix<double,6,1> ikJointVel;
        Matrix<double,6,1> EndVel;
        Matrix<double,6,1> EndPos;
        jointposition=&posR[0];
        jointvelocity=&jointRstate.velocity[0];//还未修改
        ft3wrench=&ftRmsg[2].force.x;

        thread threadRendFK([&](){
            EndPos = master_9l3_pos_FK(RorL,jointposition);
            masterR_EndPos.linear.x=EndPos[0];
            masterR_EndPos.linear.y=EndPos[1];
            masterR_EndPos.linear.z=EndPos[2];
            masterR_EndPos.angular.x=EndPos[3];
            masterR_EndPos.angular.x=EndPos[4];
            masterR_EndPos.angular.x=EndPos[5];
            masterR_EndPos_pub.publish(masterR_EndPos);
            EndVel = master_9l3_vel_FK(RorL,jointposition,jointvelocity);
            masterR_EndVel.linear.x=EndVel[0];
            masterR_EndVel.linear.y=EndVel[1];
            masterR_EndVel.linear.z=EndVel[2];
            masterR_EndVel.angular.x=EndVel[3];
            masterR_EndVel.angular.y=EndVel[4];
            masterR_EndVel.angular.z=EndVel[5];
            masterR_EndVel_pub.publish(masterR_EndVel);
        });

        ikJointVel = master_9l3_vel_IK(RorL,jointposition,ft3wrench);
         //只用控制2 3 5 6 7 8
        // ti5actuator->setVelocity(motorR[2].id,motorR[2].channel,ikJointVel[0]);
        // ti5actuator->setVelocity(motorR[3].id,motorR[3].channel,ikJointVel[1]);
        // ti5actuator->setVelocity(motorR[5].id,motorR[5].channel,ikJointVel[2]);
        // ti5actuator->setVelocity(motorR[6].id,motorR[6].channel,ikJointVel[3]);
        // ti5actuator->setVelocity(motorR[7].id,motorR[7].channel,ikJointVel[4]);
        // ti5actuator->setVelocity(motorR[8].id,motorR[8].channel,ikJointVel[5]);
        threadRendFK.join();

    });

    thread threadHeteroLctrVel([&](){
        double *jointposition;
        double *jointvelocity;
        double *ft3wrench;
        Matrix<double,6,1> ikJointVel;
        Matrix<double,6,1> EndVel;
        Matrix<double,6,1> EndPos;
        jointposition=&posL[0];
        jointvelocity=&jointLstate.velocity[0];//还未修改,要都整一下，包括正负号，角度弧度
        ft3wrench=&ftLmsg[2].force.x;
        thread threadLendFK([&](){ 
            EndPos = master_9l3_pos_FK(RorL,jointposition);
            masterL_EndPos.linear.x=EndPos[0];
            masterL_EndPos.linear.y=EndPos[1];
            masterL_EndPos.linear.z=EndPos[2];
            masterL_EndPos.angular.x=EndPos[3];
            masterL_EndPos.angular.x=EndPos[4];
            masterL_EndPos.angular.x=EndPos[5];
            masterL_EndPos_pub.publish(masterL_EndPos);
            EndVel = master_9l3_vel_FK(RorL,jointposition,jointvelocity);
            masterL_EndVel.linear.x=EndVel[0];
            masterL_EndVel.linear.y=EndVel[1];
            masterL_EndVel.linear.z=EndVel[2];
            masterL_EndVel.angular.x=EndVel[3];
            masterL_EndVel.angular.y=EndVel[4];
            masterL_EndVel.angular.z=EndVel[5];
            masterL_EndVel_pub.publish(masterL_EndVel);
    });
        ikJointVel = master_9l3_vel_IK(RorL,jointposition,ft3wrench);
        //只用控制2 3 5 6 7 8
        // ti5actuator->setVelocity(motorL[2].id,motorL[2].channel,ikJointVel[0]);
        // ti5actuator->setVelocity(motorL[3].id,motorL[3].channel,ikJointVel[1]);
        // ti5actuator->setVelocity(motorL[5].id,motorL[5].channel,ikJointVel[2]);
        // ti5actuator->setVelocity(motorL[6].id,motorL[6].channel,ikJointVel[3]);
        // ti5actuator->setVelocity(motorL[7].id,motorL[7].channel,ikJointVel[4]);
        // ti5actuator->setVelocity(motorL[8].id,motorL[8].channel,ikJointVel[5]);
        threadLendFK.join();
    });

    threadHeteroRctrVel.join();
    threadHeteroLctrVel.join();

}

void nubot_exo_node::pause(){
        for (int i = 0; i < 9; i++)
        {
            ti5actuator->setVelocity(motorR[i],0);
            //ti5actuator->setVelocity(motorL[i],0);
        }
            this_thread::sleep_for(std::chrono::milliseconds(1000));
         
            double *currentPostionL;
            double *currentPostionR;
            
            currentPostionR=ti5actuator->getPosition_batch(0,exoRmotorID,0);

            ti5actuator->setPosition_batch(0,exoRmotorID,0,currentPostionR,9);

//            currentPostionL=ti5actuator->getPosition_batch(1,exoLmotorID,1);

 //           ti5actuator->setPosition_batch(1,exoLmotorID,0,currentPostionL,9);
            
            mode.data=PAUSE;
            mode_pub.publish(mode); 
}

void nubot_exo_node::chooseMode(){
        char press=cin.get();
        if (press=='m')
        {   
            cin.get();  //消除换行符
            mode.data=HETERO;            
            cout << "HETERO模式：需要回到异构舒适操作位置请按m，若直接在当前位置进行异构操作按其他键即可" << endl;
            this_thread::sleep_for(std::chrono::milliseconds(1000));
            char press2=cin.get();
            if (press2=='m')
            {
                cin.get();  //消除换行符
                exo2initialPos();
                exoFinitialPos2comfortPos();
            }
        }
        else if (press=='b')
        {
            cin.get();  
            mode.data=HOMO;
            cout << "HOMO模式" << endl;
            this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        else
        {
            mode.data=CLAMUP;
            mode_pub.publish(mode); 
            cout << "CLAMUP模式" << endl;
            this_thread::sleep_for(std::chrono::milliseconds(1000));
        }
        cout << "模式选择完毕，按任意键开始操作，在操作过程中按n键则可暂停继续回到模式选择" << endl;
        cin.get();  //按任意键下一步
        mode_pub.publish(mode);
}
void nubot_exo_node::teleope(){
    while(cf)
    {
        // if (read(STDIN_FILENO, &ch, 1) > 0)
        // {
        //         if (ch == 'n') 
        //         {
        //             key_pressed = true;
        //             pause();
        //             chooseMode();
        //             ch=0;
        //         } 
        //         else
        //         {
        //             key_pressed = false;
        //         }
        // }
        if(mode.data==CLAMUP)
        break;
        else
        {
            waitCount++;
            if (GetValue()>=T_sample)//800Hz
                {
                //cout << "\033c";//清空屏幕
                cout << "loop time: " << setprecision(5) << GetValue() << " | T_sample: " << setprecision(5) << T_sample << endl;
                Clear();
                //批量获取力传感器信息
                // future<double(*)[6]> exoft1= async(&ftsensorPerceptron::getFTdata_batch, &*kw6ftsensor, exoRsensorID, 0 ,3);
                // future<double(*)[6]> exoft2 = async(&ftsensorPerceptron::getFTdata_batch, &*kw6ftsensor, exoLsensorID, 1 ,3);
                // ftR=exoft1.get();
                // ftL=exoft2.get();
/*采集信号*/
        std::thread threadMotorRbatch([&]() {
            double *motorR_PosVelCur;   
            for (int i = 0; i < 6; i++)
            {          
                motorR_PosVelCur=ti5actuator->getCurVelPos_thread(motorR[i]);
                motorRstate.position[i]=*(motorR_PosVelCur);
                motorRstate.velocity[i]=*(motorR_PosVelCur+1);
                motorRstate.effort[i]=*(motorR_PosVelCur+2);
                delete[] motorR_PosVelCur;
                motorR_PosVelCur=NULL;
                this_thread::sleep_for(std::chrono::microseconds(600));
            }
        });

    std::thread threadSensorRbatch([&]() {
        // this_thread::sleep_for(std::chrono::microseconds(1000));
        double *motorR_PosVelCur1;   
        for (int i = 6; i < 9; i++)
        {          
            motorR_PosVelCur1=ti5actuator1->getCurVelPos_thread(motorR[i]);//用了新对象
            motorRstate.position[i]=*(motorR_PosVelCur1);
            motorRstate.velocity[i]=*(motorR_PosVelCur1+1);
            motorRstate.effort[i]=*(motorR_PosVelCur1+2);
            delete[] motorR_PosVelCur1;
            motorR_PosVelCur1=NULL;
            this_thread::sleep_for(std::chrono::microseconds(400));
        }
        this_thread::sleep_for(std::chrono::microseconds(400));
        double *FtR;
        for (int i = 0; i < 3; i++)
        {      
            FtR=kw6ftsensor->getFTdata(sensorR[i]);

            for (int j  = 0; j < 6; j++)
            {
                ForceR[i][j]=*(FtR+j);                
            }
            ftRmsg[i].force.x=(ForceR[i][0]-ftR_offset[i][0])*9.81;
            ftRmsg[i].force.y=(ForceR[i][1]-ftR_offset[i][1])*9.81;
            ftRmsg[i].force.z=(ForceR[i][2]-ftR_offset[i][2])*9.81;
            ftRmsg[i].torque.x=(ForceR[i][3]-ftR_offset[i][3])*9.81;
            ftRmsg[i].torque.y=(ForceR[i][4]-ftR_offset[i][4])*9.81;
            ftRmsg[i].torque.z=(ForceR[i][5]-ftR_offset[i][5])*9.81;
            //力传感器值的安全判断
            if (ftRmsg[i].force.x>50) {cout <<"【警告】ftRmsg["<< i << "].force.x=" << ftRmsg[i].force.x << " 超过了50N"<< endl; ftRmsg[i].force.x=ftRmsg[i].force.x/(abs(ftRmsg[i].force.x)/50);}
            if (ftRmsg[i].force.y>50) {cout <<"【警告】ftRmsg["<< i << "].force.y=" << ftRmsg[i].force.y << " 超过了50N"<< endl; ftRmsg[i].force.y=ftRmsg[i].force.y/(abs(ftRmsg[i].force.y)/50);}
            if (ftRmsg[i].force.z>50) {cout <<"【警告】ftRmsg["<< i << "].force.z=" << ftRmsg[i].force.z << " 超过了50N"<< endl; ftRmsg[i].force.z=ftRmsg[i].force.z/(abs(ftRmsg[i].force.z)/50);}
            if (ftRmsg[i].torque.x>5) {cout <<"【警告】ftRmsg["<< i << "].torque.x=" << ftRmsg[i].torque.x << " 超过了5Nm"<< endl; ftRmsg[i].torque.x=ftRmsg[i].torque.x/(abs(ftRmsg[i].torque.x)/5);}
            if (ftRmsg[i].torque.y>5) {cout <<"【警告】ftRmsg["<< i << "].torque.y=" << ftRmsg[i].torque.y << " 超过了5Nm"<< endl; ftRmsg[i].torque.y=ftRmsg[i].torque.y/(abs(ftRmsg[i].torque.y)/5);}
            if (ftRmsg[i].torque.z>5) {cout <<"【警告】ftRmsg["<< i << "].torque.z=" << ftRmsg[i].torque.z << " 超过了5Nm"<< endl; ftRmsg[i].torque.z=ftRmsg[i].torque.z/(abs(ftRmsg[i].torque.z)/5);}

            this_thread::sleep_for(std::chrono::microseconds(800));      
        }        
            FTsensorR_shoulder_pub.publish( ftRmsg[0]);
            FTsensorR_elbow_pub.publish( ftRmsg[1]);
            FTsensorR_wrist_pub.publish( ftRmsg[2]);
        });
    std::thread threadMotorLbatch([&]() {
        double *motorL_PosVelCur;   
        for (int i = 0; i < 6; i++)
        {          
            //motorL_PosVelCur=ti5actuator2->getCurVelPos(motorL[i]);//
            motorLstate.position[i]=*(motorL_PosVelCur);
            motorLstate.velocity[i]=*(motorL_PosVelCur+1);
            motorLstate.effort[i]=*(motorL_PosVelCur+2);
            this_thread::sleep_for(std::chrono::microseconds(600));
        }
    });
        // this_thread::sleep_for(std::chrono::microseconds(1000));
    std::thread threadSensorLbatch([&]() {
        double *motorL_PosVelCur;   
        for (int i = 6; i < 9; i++)
        {          
            //motorL_PosVelCur=ti5actuator3->getCurVelPos(motorL[i]);
            motorLstate.position[i]=*(motorL_PosVelCur);
            motorLstate.velocity[i]=*(motorL_PosVelCur+1);
            motorLstate.effort[i]=*(motorL_PosVelCur+2);
            this_thread::sleep_for(std::chrono::microseconds(400));
        }
        this_thread::sleep_for(std::chrono::microseconds(400));   
        double *FtL;
        for (int i = 0; i < 3; i++)
        {                 
            //FtL=kw6ftsensor1->getFTdata(sensorL[i]); //用新的对象
            for (int j  = 0; j < 6; j++)
            {
                ForceL[i][j]=*(FtL+j);
            }
            ftLmsg[i].force.x=(ForceL[i][0]-ftL_offset[i][0])*9.81;
            ftLmsg[i].force.y=(ForceL[i][1]-ftL_offset[i][1])*9.81;
            ftLmsg[i].force.z=(ForceL[i][2]-ftL_offset[i][2])*9.81;
            ftLmsg[i].torque.x=(ForceL[i][3]-ftL_offset[i][3])*9.81;
            ftLmsg[i].torque.y=(ForceL[i][4]-ftL_offset[i][4])*9.81;
            ftLmsg[i].torque.z=(ForceL[i][5]-ftL_offset[i][5])*9.81;
            //力传感器值的安全判断
            if (abs(ftLmsg[i].force.x)>50) {cout <<"【警告】ftLmsg["<< i << "].force.x=" << ftLmsg[i].force.x << " 超过了50N"<< endl; ftLmsg[i].force.x=ftLmsg[i].force.x/(abs(ftLmsg[i].force.x)/50);}
            if (abs(ftLmsg[i].force.y)>50) {cout <<"【警告】ftLmsg["<< i << "].force.y=" << ftLmsg[i].force.y << " 超过了50N"<< endl; ftLmsg[i].force.y=ftLmsg[i].force.y/(abs(ftLmsg[i].force.y)/50);}
            if (abs(ftLmsg[i].force.z)>50) {cout <<"【警告】ftLmsg["<< i << "].force.z=" << ftLmsg[i].force.z << " 超过了50N"<< endl; ftLmsg[i].force.z=ftLmsg[i].force.z/(abs(ftLmsg[i].force.z)/50);}
            if (abs(ftLmsg[i].torque.x)>5) {cout <<"【警告】ftLmsg["<< i << "].torque.x=" << ftLmsg[i].torque.x << " 超过了5Nm"<< endl; ftLmsg[i].torque.x=ftLmsg[i].torque.x/(abs(ftLmsg[i].torque.x)/5);}
            if (abs(ftLmsg[i].torque.y)>5) {cout <<"【警告】ftLmsg["<< i << "].torque.y=" << ftLmsg[i].torque.y << " 超过了5Nm"<< endl; ftLmsg[i].torque.y=ftLmsg[i].torque.y/(abs(ftLmsg[i].torque.y)/5);}
            if (abs(ftLmsg[i].torque.z)>5) {cout <<"【警告】ftLmsg["<< i << "].torque.z=" << ftLmsg[i].torque.z << " 超过了5Nm"<< endl; ftLmsg[i].torque.z=ftLmsg[i].torque.z/(abs(ftLmsg[i].torque.z)/5);}
        }
            this_thread::sleep_for(std::chrono::microseconds(800));       
            FTsensorL_shoulder_pub.publish( ftLmsg[0]);
            FTsensorL_elbow_pub.publish( ftLmsg[1]);
            FTsensorL_wrist_pub.publish( ftLmsg[2]);
        });     
        threadSensorRbatch.join();
        threadMotorRbatch.join();

        threadMotorLbatch.join();
        threadSensorLbatch.join();
        
        if (mode.data==STOP)
        {
            mode_pub.publish(mode);//如果通讯中断发布stop命令
            cin.get();
        }
        cout << "电机信号采集用时:" << GetValue() << endl;
        for (int i = 0; i < 9; i++)
        {
            cout<< "ExosJoint: " 
            <<" |pos " << "R" << i << ": " << setw(10) << motorRstate.position[i] <<" L" << i << ": "  << setw(10) << motorLstate.position[i] 
            << "|vel "  << "R" << i << ": " << setw(10) << motorRstate.velocity[i] << " L"  << i << ": "  << setw(10)<< motorLstate.velocity[i] 
            << "|cur "  << "R" << i << ": " << setw(10) << motorRstate.effort[i]  << " L"  << i << ": " << setw(10)<< motorLstate.effort[i] 
            << endl;  
        }
        // if(abs(motorRstate.position[1])>10)
        // cin.get();
        for (int i = 0; i < 3; i++)
        {
            //printf("fx= %2f Kg,fy= %2f Kg,fz= %2f Kg,mx= %2f KgM,my= %2f KgM,mz= %2f KgM\n",ForceR[i][0],ForceR[i][1],ForceR[i][2],ForceR[i][3],ForceR[i][4],ForceR[i][5]);
            printf("exoRsensor: Fx= %10f N,Fy= %10f N,Fz= %10f N,Tx= %10f NM,Ty= %10f NM,Tz= %10f NM\n", ftRmsg[i].force.x, ftRmsg[i].force.y, ftRmsg[i].force.z,ftRmsg[i].torque.x,ftRmsg[i].torque.y,ftRmsg[i].torque.z);
            //printf("exoLsensor: Fx= %10f N,Fy= %10f N,Fz= %10f N,Tx= %10f NM,Ty= %10f NM,Tz= %10f NM\n", ftLmsg[i].force.x, ftLmsg[i].force.y, ftLmsg[i].force.z,ftLmsg[i].torque.x,ftLmsg[i].torque.y,ftLmsg[i].torque.z);
        }

        // this_thread::sleep_for(std::chrono::microseconds(1000));
        motor2joint(); //将电机信息转换成关节信息并发布
        cout << "FT sample time: " << setprecision(5) << GetValue() << endl;         
        cout << "Actuator sample time: " << setprecision(5) << GetValue() << endl;
            /*开始控制*/

            for (int i = 0; i < 3; i++)
            {
                ForceRx[i]=ftRmsg[i].force.x*1000;
                ForceRy[i]=ftRmsg[i].force.y*1000;
                ForceRz[i]=ftRmsg[i].force.z*1000;
                Torquex[i]=ftRmsg[i].torque.x*1000;
                TorqueRy[i]=ftRmsg[i].torque.y*1000;
                TorqueRz[i]=ftRmsg[i].torque.z*1000;

                ForceLx[i]=ftRmsg[i].force.x*1000;
                ForceLy[i]=ftRmsg[i].force.y*1000;
                ForceLz[i]=ftRmsg[i].force.z*1000;
                TorqueLx[i]=ftRmsg[i].torque.x*1000;
                TorqueLy[i]=ftRmsg[i].torque.y*1000;
                TorqueLz[i]=ftRmsg[i].torque.z*1000;
            }                    
            for (int i = 0; i < 9; i++)
            {
                posR[i]=motorRstate.position[i]/180*pi;//用弧度表示
                posL[i]=motorLstate.position[i]/180*pi;
            }
            posR[3]=-posR[3];//3关节取反
            posL[3]=-posL[3];
            //加平滑加冗余过滤加力矩通道大臂与肱骨对齐
            if(mode.data==HOMO)
            masterHomo();
            if(mode.data==HETERO)
            masterHetero(1);             
            
            loop_count++;
            cout << "【step】: " << loop_count << "   Void loop number: " << waitCount << "   caculation cost time: " << setprecision(5) << GetValue() << endl;
            waitCount=0;   
            }
                
        }
        ros::spinOnce();
    }
}
        
                   
void nubot_exo_node::motor2joint()//转成弧度
{  
            for (int i = 0; i < 9; i++)
            {   
                if((i==3) || (i==4) || (i==6) || (i==7))
                {
                jointRstate.position[i] =-motorRstate.position[i]*pi/180;
                jointRstate.velocity[i] = -motorRstate.velocity[i]*pi/180;
                jointRstate.effort[i] = -motorRstate.effort[i];

                jointLstate.position[i] =-motorLstate.position[i]*pi/180;
                jointLstate.velocity[i] = -motorLstate.velocity[i]*pi/180;
                jointLstate.effort[i] = -motorLstate.effort[i];
                }
                else
                {
                jointRstate.position[i] =motorRstate.position[i]*pi/180;
                jointRstate.velocity[i] =motorRstate.velocity[i]*pi/180;
                jointRstate.effort[i] =motorRstate.effort[i];

                jointLstate.position[i] =motorLstate.position[i]*pi/180;
                jointLstate.velocity[i] =motorLstate.velocity[i]*pi/180;
                jointLstate.effort[i] =motorLstate.effort[i];
                }   
            }
            masterR_Joint_pub.publish(jointRstate);
            masterL_Joint_pub.publish(jointLstate);
}


int main(int argc, char *argv[])
{
   
    ros::init(argc, argv, "rafiki");

    nubot_exo_node  rafiki;

    return 0;
}




