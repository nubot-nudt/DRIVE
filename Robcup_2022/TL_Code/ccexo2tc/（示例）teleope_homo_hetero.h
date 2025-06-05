#ifndef TELEOPE_HOMO_HETERO_H
#define TELEOPE_HOMO_HETERO_H
#include <iostream>
#include <vector>
#include <cmath>
#include <Eigen/Dense>

#define TELEOPE_MODE 1
#define JOINT_MODE 1

#define pi 3.1415926
using namespace std;
using namespace Eigen;

class teleope_homo_hetero
{
private:
enum EXOMAN_FLAG
{
    EXO_R=1,
    EXO_L=1,
    MAN_R=1,
    MAN_L=1
};
    struct DH_param_S
    {
        double theta;
        double  d;
        double  a;
        double  alpha;
    };
    DH_param_S exo_DH_L[9];
    DH_param_S exo_DH_R[9];
    DH_param_S mani_DH_L[7];
    DH_param_S mani_DH_R[7];

    double  a1=0.15;//其实不用改
    double  d2=0.225;
    double  d4=0.32;
    double  d7=0.26;
    double  a8=0.08;

    double exoLtheta[9]=  {0,            pi/2,      pi/2,         -pi/2,          0,                pi/2,        pi/2,        -pi/2,          0};
    double  exoLd[9]=         {0,            0,            d2,              0,                d4,             0,             d7,             0,               0};
    double  exoLa[9]=         {0,            a1,            0,                0,                0,                0,             0,               0,             a8};
    double  exoLalpha[9]={0,           pi/2,       -pi/2,          pi/2,         -pi/2,        pi/2,        -pi/2,        -pi/2,         0};

    double  exoRtheta[9]=  {0,          pi/2,      pi/2,         pi/2,          0,                -pi/2,        -pi/2,        pi/2,          0};
    double  exoRd[9]=         {0,            0,            d2,              0,                d4,             0,             d7,             0,               0};
    double  exoRa[9]=         {0,          -a1,            0,                0,                0,                0,             0,               0,             a8};
    double  exoRalpha[9]={0,           pi/2,       pi/2,         -pi/2,         pi/2,        -pi/2,        -pi/2,        pi/2,         0};

 

    double dx = 0, dy = 0, dz = 0, drx=0, dry=0, drz=0, dpitch = 0, dyaw = 0, droll = 0, dax = 0, day = 0, daz = 0;
    double dnx = 0, dny = 0, dnz = 0, dpx = 0, dpy = 0, dpz = 0, dox = 0, doy = 0, doz = 0;
    double roll, pitch, yaw;
    double dtheta[9] = {0, 0, 0, 0, 0, 0, 0,0}; //末端控制状态控制速度 ，下标从1开始，theta[0]无用。单位弧度。
    double theta[9] = {0, 0, 0, 0, 0, 0, 0,0};  //末端控制状态角度 ，下标从1开始，theta[0]无用。单位弧度。
    double nx, ox, ax, px, ny, oy, ay, py, nz, oz, az, pz;


public:
    teleope_homo_hetero(/* args */);
    ~teleope_homo_hetero();
    
//基本操作
        Matrix4d TransMatrix(double joint,double theta,double d,double a,double alpha);
        
        // void pause();
        // void master2initialPosion();
        // void master2comfotablePosion();
        // void slave2initialPosition();


    //同构

        //主臂
    Vector3d  master_shoulder_pos_FK(bool RorL,double theta0,double theta1,double theta2,double theta3,double theta4);//主臂肩部位置层正运动学                提供肩部主从控制位置闭环
    Vector3d master_shoulder_staticForce_IK(bool RorL,double *theta,double *dtheta,double *Force);//主臂肩部静力逆运动学      控制主臂肩部运动
    Vector3d master_shoulder_vel_FK(bool RorL,double theta0,double theta1,double theta2,double theta3,double theta4,double dtheta0,double dtheta1,double dtheta2,double dtheta3,double dtheta4);//主臂肩部速度层正运动学                控制主臂肱骨运动
        
        //从臂
    double slave_shoulder_vel_IK(); //从臂肩部速度层逆运动学            控制从臂肩部运动
    double slave_shoulder_pos_FK();//从臂肩部位置层正运动学          提供从臂肩部主从控制位置闭环
   
    //异构

        //主臂
        Matrix<double,6,1> master_9l3_pos_FK(bool RorL,double *theta);//从臂位置层正运动学                        提供主从控制主臂末端位置闭环
        Matrix<double,6,1> master_9l3_vel_FK(bool RorL,double *theta,double *dtheta);//从臂速度层正运动学                         传递主臂末端速度控制量给从臂末端
        Matrix<double,6,1> master_9l3_vel_IK(bool RorL,double *theta,double *wrench);//从臂锁3自由度速度层逆运动学      控制主臂异构下的运动
        
        //从臂
        double slave_7l1_vel_IK();//从臂锁1自由度速度层逆运动学           控制从臂异构下的运动
        double slave_pos_FK();//从臂位置层正运动学                                     提供异构主从控制从臂位置闭环

//切换
         
        void homo2hetero();//是否回到舒适区域   都先pause//homo2hetero_Direct homo2hetero_Comfort


        void hetero2homo();//主臂回到与从臂同样位姿，homo2hetero_Direct homo2hetero_Comfort
        // {double  getSlaveJontPosition();
        // double getMasterJointPosition();
        // }
//调节
        void adjustTeleopeProportion(); 

        
        //同构切换为异构
            //(1)不回到初始位置直接开始操作，主臂——>从臂
            //(2)从臂不动，主臂回到舒适操作位置


//暂停

//结束
};















#endif // TELEOPE_HOMO_HETERO_H