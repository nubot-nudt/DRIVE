#include "teleope_homo_hetero.h"


Matrix4d teleope_homo_hetero::TransMatrix(double joint,double theta,double d,double a,double alpha)
{
    theta=joint+theta;
Matrix4d A;
                    A << cos(theta), -sin(theta)*cos(alpha),   sin(theta)*sin(alpha)   ,   a*cos(theta),
                             sin(theta)  , cos(theta)*cos(alpha) ,  -cos(theta)*sin(alpha)  ,  a*sin(theta),
                                0                 ,sin(alpha)                         , cos(alpha)                           ,  d                      ,
                                0                 ,0                                            , 0                                              ,  1                      ;
return A;
}

teleope_homo_hetero::teleope_homo_hetero()
{   
    for (int i = 0; i < 9; i++)
    {
        exo_DH_L[i].theta=exoLtheta[i];
        exo_DH_L[i].d=exoLd[i];
        exo_DH_L[i].a=exoLa[i];
        exo_DH_L[i].alpha=exoLalpha[i];

        exo_DH_R[i].theta=exoRtheta[i];
        exo_DH_R[i].d=exoRd[i];
        exo_DH_R[i].a=exoRa[i];
        exo_DH_R[i].alpha=exoRalpha[i];
    }
}

teleope_homo_hetero::~teleope_homo_hetero()
{
}

Vector3d teleope_homo_hetero::master_shoulder_pos_FK(bool RorL,double theta0,double theta1,double theta2,double theta3,double theta4)
{
MatrixXd T[5];
Matrix4d A[5]; 

double theta[5]={theta0,theta1,theta2,theta3,theta4};

    A[0]<<1,0, 0 ,0,
                0, cos(theta0),-sin(theta0),0,
                0, sin(theta0), cos(theta0), 0,
                0,0,0,1;
if (RorL)
{
    for (int i = 1; i < 5; i++)
    {
        A[i]=TransMatrix(theta[i],exo_DH_L[i].theta,exo_DH_L[i].d,exo_DH_L[i].a,exo_DH_L[i].alpha);
    }
}
else
        for (int i = 1; i < 5; i++)
    {
        A[i]=TransMatrix(theta[i],exo_DH_R[i].theta,exo_DH_R[i].d,exo_DH_R[i].a,exo_DH_R[i].alpha);
    }
T[0]=A[0];
for (int i = 1; i < 5; i++)
{
    T[i]=T[i-1]*A[i];
}
Matrix3d rotation_vector=T[4].block<3,3>(0,0);//从位姿矩阵中提取旋转矩阵
//Vector3d eulerAngle=rotation_vector.matrix().eulerAngles(0,1,2);//RPY roll-x pitch-y yaw-z
Vector3d eulerAngle;//旋转矩阵转化为RPY
//相对于基座固定坐标系的姿态，外旋先绕rx->ry->rz
// eulerAngle(0) = std::atan2(rotation_vector(2, 1), rotation_vector(2, 2));
// eulerAngle(1) = std::atan2(-rotation_vector(2, 0), std::sqrt(rotation_vector(2, 1) * rotation_vector(2, 1) + rotation_vector(2, 2) * rotation_vector(2, 2)));
// eulerAngle(2) = std::atan2(rotation_vector(1, 0), rotation_vector(0, 0));
//相对于基座运动坐标系的姿态，内旋先绕rx->ry->rz
eulerAngle(0) = std::atan2(-rotation_vector(1, 2), rotation_vector(2, 2));
eulerAngle(1) = std::atan2(rotation_vector(0, 2), std::sqrt(rotation_vector(0, 0) * rotation_vector(0, 0) + rotation_vector(0, 1) * rotation_vector(0, 1)));
eulerAngle(2) = std::atan2(-rotation_vector(0, 1), rotation_vector(0, 0));
return eulerAngle;
}

Vector3d teleope_homo_hetero::master_shoulder_vel_FK(bool RorL,double theta0,double theta1,double theta2,double theta3,double theta4,double dtheta0,double dtheta1,double dtheta2,double dtheta3,double dtheta4)
{
    //MatrixXd T[4];
    Matrix4d A[5]; 
    Matrix4d AT[5]; 
    MatrixXd J=MatrixXd::Zero(6,5);   
    double theta[5]={theta0,theta1,theta2,theta3,theta4};
    VectorXd dtheta(5) ;
    dtheta << dtheta0,dtheta1,dtheta2,dtheta3,dtheta4;
        A[0]<<1,0, 0 ,0,
                    0, cos(theta0),-sin(theta0),0,
                    0, sin(theta0), cos(theta0), 0,
                    0,0,0,1;
    if (RorL)
    {
        for (int i = 1; i < 5; i++)
        {
            A[i]=TransMatrix(theta[i],exo_DH_L[i].theta,exo_DH_L[i].d,exo_DH_L[i].a,exo_DH_L[i].alpha);
        }
    }
    else
            for (int i = 1; i < 5; i++)
        {
            A[i]=TransMatrix(theta[i],exo_DH_R[i].theta,exo_DH_R[i].d,exo_DH_R[i].a,exo_DH_R[i].alpha);
        }
    Matrix4d  at=MatrixXd::Identity(4,4);
    for (int i=0;i<5;i++)
    {   
        at=A[4-i]*at;
        AT[4-i]=at;
    }
    for(int i=0;i<5;i++)
    {
        if (i==0)//绕x轴与书上不一样
        {
            J(0,i)=-AT[i](1,0)*AT[i](2,3)+AT[i](2,0)*AT[i](1,3);
            J(1,i)=-AT[i](1,1)*AT[i](2,3)+AT[i](2,1)*AT[i](1,3);
            J(2,i)=-AT[i](1,2)*AT[i](2,3)+AT[i](2,2)*AT[i](1,3);
            J(3,i)=AT[i](0,0);
            J(4,i)=AT[i](0,1);
            J(5,i)=AT[i](0,2);
        }
        else
        {
            J(0,i)=-AT[i](0,0)*AT[i](1,3)+AT[i](1,0)*AT[i](0,3);
            J(1,i)=-AT[i](0,1)*AT[i](1,3)+AT[i](1,1)*AT[i](0,3);
            J(2,i)=-AT[i](0,2)*AT[i](1,3)+AT[i](1,2)*AT[i](0,3);
            J(3,i)=AT[i](2,0);
            J(4,i)=AT[i](2,1);
            J(5,i)=AT[i](2,2);
        }
    }
    VectorXd pose(6);
    pose=J*dtheta;
    return pose.segment(3,3);// 取向量第i到第i+n个元素，如vec.segment(0,3)，取前三个
}

Vector3d teleope_homo_hetero::master_shoulder_staticForce_IK(bool RorL,double *theta,double *dtheta,double *Force)
 {     //剔除干扰力的静力运动学
        //joint3运动反向
        //1.5mA的电流产生1N的力
        Vector3cd Tt_fromFT;
        float d2=0.22;
        float a3=0.15;
        double Forcex=Force[0];
        double Forcey=Force[1];
        double Forcez=Force[2];
        double Torquex=Force[3];
        double Torquey=Force[4];
        double Torquez=Force[5];
        if (RorL)
        {
        Tt_fromFT[0]=cos(theta[2])*(d2+a3*sin(theta[3]))*Forcez - cos(theta[2])*sin(theta[3])*Torquey;
        Tt_fromFT[1]=a3*Forcez*cos(theta[3]) +cos(theta[3])*Torquey;
        Tt_fromFT[2]=a3*Forcey-Torquez;
        }
        else
        {
        Tt_fromFT[0]=-cos(theta[2])*(d2+a3*sin(theta[3]))*Forcez + cos(theta[2])*sin(theta[3])*Torquey;
        Tt_fromFT[1]=-a3*Forcez*cos(theta[3]) + cos(theta[3])*Torquey;
        Tt_fromFT[2]=a3*Forcey-Torquez;
        }
 }

Matrix<double,6,1> teleope_homo_hetero::master_9l3_pos_FK(bool RorL,double *theta)
{
    MatrixXd T[9];
    Matrix4d A[9]; 

        A[0]<<1,0, 0 ,0,
                    0, cos(theta[0]),-sin(theta[0]),0,
                    0, sin(theta[0]), cos(theta[0]), 0,
                    0,0,0,1;
    if (RorL)
    {
        for (int i = 1; i < 9; i++)
        {
            A[i]=TransMatrix(theta[i],exo_DH_L[i].theta,exo_DH_L[i].d,exo_DH_L[i].a,exo_DH_L[i].alpha);
        }
    }
    else
            for (int i = 1; i < 9; i++)
        {
            A[i]=TransMatrix(theta[i],exo_DH_R[i].theta,exo_DH_R[i].d,exo_DH_R[i].a,exo_DH_R[i].alpha);
        }
    T[0]=A[0];
    for (int i = 1; i < 9; i++)
    {
        T[i]=T[i-1]*A[i];
    }
    Matrix3d rotation_vector=T[8].block<3,3>(0,0);//从位姿矩阵中提取旋转矩阵
    Vector3d eulerAngle=rotation_vector.matrix().eulerAngles(0,1,2);//RPY roll-x pitch-y yaw-z
    Vector3d point=T[8].block<3,1>(0,3);
    Matrix<double,6,1> pose;
    pose<<point,eulerAngle;
    return pose;
}

Matrix<double,6,1> teleope_homo_hetero::master_9l3_vel_FK(bool RorL,double *theta,double *dtheta)
{
    Matrix4d A[9]; 
    Matrix4d AT[9]; 
    Matrix4d A_9l3[6]; 
    Matrix4d AT_9l3[6];
    MatrixXd J(6,9);
    MatrixXd J_9l3(6,6);
    Matrix<double,9,1> dtheta_m;
    Matrix<double,6,1> dtheta_9l3;
    dtheta_m << dtheta[0],dtheta[1],dtheta[2],dtheta[3],dtheta[4],dtheta[5],dtheta[6],dtheta[7],dtheta[8];
    dtheta_9l3 << dtheta[2],dtheta[3],dtheta[5],dtheta[6],dtheta[7],dtheta[8];
        A[0]<<1,0, 0 ,0,
                    0, cos(theta[0]),-sin(theta[0]),0,
                    0, sin(theta[0]), cos(theta[0]), 0,
                    0,0,0,1;
    if (RorL)
    {
        for (int i = 1; i < 9; i++)
        {
            A[i]=TransMatrix(theta[i],exo_DH_L[i].theta,exo_DH_L[i].d,exo_DH_L[i].a,exo_DH_L[i].alpha);
       }
    }
    else
            for (int i = 1; i < 10; i++)
        {
            A[i]=TransMatrix(theta[i],exo_DH_R[i].theta,exo_DH_R[i].d,exo_DH_R[i].a,exo_DH_R[i].alpha);
        }

    Matrix4d  at=MatrixXd::Identity(4,4);
    
    for (int i=0;i<9;i++)
    {   
        at=A[8-i]*at;
        AT[8-i]=at;
    }
    A_9l3[0]= A[0]*A[1]*A[2]; 
    A_9l3[1]= A[3], 
    A_9l3[2]= A[4]*A[5], 
    A_9l3[3]= A[6], 
    A_9l3[4]= A[7], 
    A_9l3[5]= A[8];

    at=MatrixXd::Identity(4,4);    
        for (int i=0;i<6;i++)
    {   
        at=A_9l3[5-i]*at;
        AT_9l3[5-i]=at;
    }
    for(int i=0;i<9;i++)//相对与末端坐标系下的雅克比
    {
        if (i=0)//绕x轴与书上不一样
        {
            J(0,i)=-AT[i](1,0)*AT[i](2,3)+AT[i](2,0)*AT[i](1,3);
            J(1,i)=-AT[i](1,1)*AT[i](2,3)+AT[i](2,1)*AT[i](1,3);
            J(2,i)=-AT[i](1,2)*AT[i](2,3)+AT[i](2,2)*AT[i](1,3);
            J(3,i)=AT[i](0,0);
            J(4,i)=AT[i](0,1);
            J(5,i)=AT[i](0,2);
        }
        else
        {
            J(0,i)=-AT[i](0,0)*AT[i](1,3)+AT[i](1,0)*AT[i](0,3);
            J(1,i)=-AT[i](0,1)*AT[i](1,3)+AT[i](1,1)*AT[i](0,3);
            J(2,i)=-AT[i](0,2)*AT[i](1,3)+AT[i](1,2)*AT[i](0,3);
            J(3,i)=AT[i](2,0);
            J(4,i)=AT[i](2,1);
            J(5,i)=AT[i](2,2);
        }
    }
        for(int i=0;i<6;i++)//相对与末端坐标系下的雅克比
    {
            J_9l3(0,i)=-AT_9l3[i](0,0)*AT_9l3[i](1,3)+AT_9l3[i](1,0)*AT_9l3[i](0,3);
            J_9l3(1,i)=-AT_9l3[i](0,1)*AT_9l3[i](1,3)+AT_9l3[i](1,1)*AT_9l3[i](0,3);
            J_9l3(2,i)=-AT_9l3[i](0,2)*AT_9l3[i](1,3)+AT_9l3[i](1,2)*AT_9l3[i](0,3);
            J_9l3(3,i)=AT_9l3[i](2,0);
            J_9l3(4,i)=AT_9l3[i](2,1);
            J_9l3(5,i)=AT_9l3[i](2,2);
        
    }
    VectorXd pose(6);
    pose=J_9l3*dtheta_9l3;
    return pose;// 取向量第i到第i+n个元素，如vec.segment(0,3)，取前三个
}

Matrix<double,6,1> teleope_homo_hetero::master_9l3_vel_IK(bool RorL,double *theta,double *wrench)
{   

    //相对于末端的微分运动
    MatrixXd T[8];
    Matrix4d A[8]; 
    Matrix4d delta;
    Matrix4d RHS;
    Matrix4d dRHS;
    double dtheta2,dtheta3,dtheta5,dtheta6,dtheta7,dtheta8;//lock joint 0,1,4
    Matrix<double,6,1> target_dtheta;
    dx=wrench[0]; dy=wrench[1]; dz=wrench[2];drx=wrench[0]; dry=wrench[1]; drz=wrench[2];
        delta<<      0,     -dyaw, dpitch,        dx,
                        dyaw,      0,          -droll,       dy,
                        -dpitch, droll,       0,             dz,
                                0,          0,          0,              0;
        droll=drx;dpitch=dry;dyaw=drz;
        
        A[0]<<1,0, 0 ,0,
                    0, cos(theta[0]),-sin(theta[0]),0,
                    0, sin(theta[0]), cos(theta[0]), 0,
                    0,0,0,1;
    if (RorL)
    {
        for (int i = 1; i < 9; i++)
        {
            A[i]=TransMatrix(theta[i],exo_DH_L[i].theta,exo_DH_L[i].d,exo_DH_L[i].a,exo_DH_L[i].alpha);
        }
    }
    else
            for (int i = 1; i < 9; i++)
        {
            A[i]=TransMatrix(theta[i],exo_DH_R[i].theta,exo_DH_R[i].d,exo_DH_R[i].a,exo_DH_R[i].alpha);
        }
    T[0]=A[0];
    for (int i = 1; i < 9; i++)
    {
        T[i]=T[i-1]*A[i];
    }
    RHS=T[8];
    
    dRHS=delta*RHS;

    nx=RHS(0,0);ox=RHS(0,1);ax=RHS(0,2);px=RHS(0,3);
    ny=RHS(1,0);oy=RHS(1,1);ay=RHS(1,2);py=RHS(1,3);
    nz=RHS(2,0);oz=RHS(2,1);az=RHS(2,2);pz=RHS(2,3);

    dnx=dRHS(0,0);dox=dRHS(0,1);dax=dRHS(0,2);dpx=dRHS(0,3);
    dny=dRHS(1,0);doy=dRHS(1,1);day=dRHS(1,2);dpy=dRHS(1,3);
    dnz=dRHS(2,0);doz=dRHS(2,1);daz=dRHS(2,2);dpz=dRHS(2,3);

        // //std::cout << "基座";

        // dnx = dpitch * nz - dyaw * ny;
        // dny = dyaw * nx - droll * nz;
        // dnz = droll * ny - dpitch * nx;

        // dox = dpitch * oz - dyaw * oy;
        // doy = dyaw * ox - droll * oz;
        // doz = droll * oy - dpitch * ox;

        // dax = az * dpitch - ay * dyaw;
        // day = ax * dyaw - az * droll;
        // daz = ay * droll - ax * dpitch;

        // dpx = dx + dpitch * pz - dyaw * py;
        // dpy = dy - droll * pz + dyaw * px;
        // dpz = dz - dpitch * px + droll * py;

        //std::cout << "末端";

        dnx = dyaw*ox - ax*dpitch;
        dny = dyaw*oy - ay*dpitch;
        dnz = dyaw*oz - az*dpitch;

        dox = ax*droll - dyaw*nx;
        doy = ay*droll - dyaw*ny;
        doz = az*droll - dyaw*nz;

        dax = dpitch*nx - droll*ox;
        day = dpitch*ny - droll*oy;
        daz = dpitch*nz - droll*oz;

        dpx = ax * dz + dx * nx + dy * ox;
        dpy = ay * dz + dx * ny + dy * oy;
        dpz = az * dz + dx * nz + dy * oz;

        float s0=sin(theta[0]);
        float s1=sin(theta[1]);
        float s2=sin(theta[2]);
        float s3=sin(theta[3]);
        float s4=sin(theta[4]);
        float s5=sin(theta[5]);
        float s6=sin(theta[6]);
        float s7=sin(theta[7]);
        float s8=sin(theta[8]);

        float c0=cos(theta[0]);
        float c1=cos(theta[1]);
        float c2=cos(theta[2]);
        float c3=cos(theta[3]);
        float c4=cos(theta[4]);
        float c5=cos(theta[5]);
        float c6=cos(theta[6]);
        float c7=cos(theta[7]);
        float c8=cos(theta[8]);


        float a1c2=a1*c2
                ,pxc2s1=px*c2*s1
                ,pzc0s2=pz*c0*s2
                ,pys0s2=py*s0*s2
                ,pyc0c1c2=py*c0*c1*c2
                ,pzs0c1c2=pz*s0*c1*c2
                ,a8nxc2s1=a8*nx*c2*s1
                ,a8nzc0s2=a8*nz*c0*s2
                ,a8nys0s2=a8*ny*s0*s2
                ,a8nyc0c1c2=a8*ny*c0*c1*c2
                ,a8nzs0c1c2=a8*nz*s0*c1*c2;
    float a1s2=a1*s2
                ,pzc0c2=pz*c0*c2
                ,pys0c2=py*s0*c2
                ,pxs1s2=px*s1*s2
                ,pyc0c1s2=py*c0*c1*s2
                ,pzs0c1s2 =pz*s0*c1*s2
                ,a8nzc0c2=a8*nz*c0*c2
                ,a8nys0c2= a8*ny*s0*c2
                ,a8nxs1s2= a8*nx*s1*s2
                ,a8nyc0c1s2= a8*ny*c0*c1*s2
                ,a8nzs0c1s2=a8*nz*s0*c1*s2;
    float pxc1=px*c1
                ,a8nxc1 =a8*nx*c1
                ,pyc0s1  =py*c0*s1
                ,pzs0s1  =pz*s0*s1
                ,a8nyc0s1 = a8*ny*c0*s1
                ,a8nzs0s1=a8*nz*s0*s1;
    float   c0s2=c0*s2
                ,s0c1c2= s0*c1*c2
                ,c0c2 =c0*c2
                ,s0c1s2 =s0*c1*s2
                ,s0s1=s0*s1
                ,s0s2=s0*s2
                ,c0c1c2=c0*c1*c2
                ,s0c2=s0*c2
                ,c0c1s2= c0*c1*s2
                ,c0s1=c0*s1
                ,s1s2= s1*s2
                ,c2s1= c2*s1;
        
        float c3c5_c4s3s5=c3*c5 - c4*s3*s5;
        float c3s5_c4c5s3=c3*s5 + c4*c5*s3;//dist71
        float c5s3_c3c4s5=c5*s3 + c3*c4*s5;//c5s3_c3c4s5 dist73
        float s3s5_c3c4c5=s3*s5 - c3*c4*c5;//-dist7
        
        float powd7s4=pow(d7,2)*pow(s4,2);
        float pow2d4d7=2*pow(d4,2)*pow(d7,2);
        float c2c4_s2s3s4;
        float c4s2_c2s3s4;
        float s1s3_c1c3s2;
        float dist2_1,dist2_2,dist7_1,dist7_2,dist7_3,dist7_4;

if(RorL)//L=1,R=0
{
    c2c4_s2s3s4=c2*c4 - s2*s3*s4;//左臂不一样
    s1s3_c1c3s2=s1*s3 + c1*c3*s2;//左臂不一样        
    c4s2_c2s3s4=c4*s2 + c2*s3*s4;//左臂不一样

    dist2_1=(a1c2 + pxc2s1 - pz*c0s2 + pys0s2 - pyc0c1c2 - pzs0c1c2 - a8nxc2s1 + a8nzc0s2 - a8nys0s2 + a8nyc0c1c2 + a8nzs0c1c2);
    dist2_2=(a1s2 + pzc0c2 - pys0c2 + pxs1s2 - pyc0c1s2 - pzs0c1s2 - a8nzc0c2 + a8nys0c2 - a8nxs1s2 + a8nyc0c1s2 + a8nzs0c1s2);
    
    dist7_1 = c2*c5*s4 + s2*c3s5_c4c5s3;
    dist7_2=-c2*c3s5_c4c5s3 + c5*s2*s4;
    dist7_3=-c2*c3c5_c4s3s5 - s2*s4*s5;
    dist7_4=-s2*c3c5_c4s3s5 + c2*s4*s5;


}
else
{
        float c2c4_s2s3s4=c2*c4 + s2*s3*s4;
        float c4s2_c2s3s4=c4*s2 - c2*s3*s4;
        float s1s3_c1c3s2=s1*s3 - c1*c3*s2;

        dist2_1=(a1c2 - pxc2s1 + pzc0s2 - pys0s2 + pyc0c1c2 + pzs0c1c2 + a8nxc2s1 - a8nzc0s2 + a8nys0s2 - a8nyc0c1c2 - a8nzs0c1c2);
        dist2_2=(a1s2 - pzc0c2 + pys0c2 - pxs1s2 + pyc0c1s2 + pzs0c1s2 + a8nzc0c2 - a8nys0c2 + a8nxs1s2 - a8nyc0c1s2 - a8nzs0c1s2);

        dist7_1=c2*c5*s4 - s2*c3s5_c4c5s3;//dist8
        dist7_2=c2*c3s5_c4c5s3 + c5*s2*s4;//dist9
        dist7_3=c2*c3c5_c4s3s5 - s2*s4*s5;//dist10
        dist7_4=s2*c3c5_c4s3s5 + c2*s4*s5;//dist11
}
       float dist2_3=(d2 - pxc1 + a8nxc1 - pyc0s1 - pzs0s1 + a8nyc0s1 + a8nzs0s1);
       float dist2_4=(pow(dist2_1,2) + pow(dist2_2,2) - pow(d4,2) - pow(d7,2) + pow(dist2_3,2))/pow2d4d7;
        
        float c0s2_s0c1c2=c0s2 + s0c1c2;
        float s0s2_c0c1c2=s0s2 - c0c1c2;
        float c0c2_s0c1s2=c0c2 - s0c1s2;
        float s0c2_c0c1s2=s0c2 + c0c1s2;
        float dist2_5=a8*c0s2_s0c1c2;//dist5 
        float dist2_6=a8*s0s2_c0c1c2;//dist6

        double dist7_5=c1*c5s3_c3c4s5 + s1*dist7_4;
        double dist7_6=c0s1*c3s5_c4c5s3 + s0c2*s3s5_c3c4c5 + c0c1s2*s3s5_c3c4c5;
        double dist7_7=c0c2*s3s5_c3c4c5 - s0s1*c3s5_c4c5s3 - s0c1s2*s3s5_c3c4c5;
        double dist7_8=s0*dist7_1 + c0*c1*dist7_2;
        double dist7_9=c0*dist7_1 - s0*c1*dist7_2;
//左臂
    if (RorL)
    {
            dtheta2=(powd7s4*(
                            dnz*((2*dist2_5*dist2_1)/powd7s4 + (2*dist2_5*dist2_1 - 2*a8*c0c2_s0c1s2*dist2_2 + 2*a8*s0s1*dist2_3)*dist2_4) - 
                            dpy*(-(2*s0s2_c0c1c2*dist2_1)/powd7s4 + (- 2*s0s2_c0c1c2*dist2_1 + 2*s0c2_c0c1s2*dist2_2 + 2*c0s1*dist2_3)*dist2_4) + 
                            dpx*((2*s1s2*dist2_2 - 2*c1*dist2_3 + 2*c2s1*dist2_1)*dist2_4 + (2*c2s1*dist2_1)/powd7s4) - 
                            dpz*((2*c0s2_s0c1c2*dist2_1 - 2*c0c2_s0c1s2*dist2_2 + 2*s0s1*dist2_3)*dist2_4 + (2*c0s2_s0c1c2*dist2_1)/powd7s4) - 
                            dnx*((2*a8*c2s1*dist2_1 - 2*a8*c1*dist2_3 + 2*a8*s1s2*dist2_2)*dist2_4 + (2*a8*c2s1*dist2_1)/powd7s4) + 
                            dny*((2*a8*s0c2_c0c1s2*dist2_2 - 2*dist2_6*dist2_1 + 2*a8*c0s1*dist2_3)*dist2_4 - (2*dist2_6*dist2_1)/powd7s4)
                            ))/(2*dist2_1*dist2_2);

             dtheta5=-(dpx*c2s1 + 
                                                dpy*s0s2_c0c1c2 - 
                                                dpz*c0s2_s0c1c2 - 
                                                dnx*a8*c2s1 - 
                                                dny*dist2_6 + 
                                                dnz*dist2_5  - 
                                                dtheta2*dist2_2
                                                )/(d7*s4*s5);

             dtheta3=-(dpx*c1 + 
                                                dpy*c0s1 + 
                                                dpz*s0s1 - 
                                                dnx*a8*c1 - 
                                                dny*a8*c0s1 - 
                                                dnz*a8*s0s1 + 
                                                dtheta5*d7*c5s3_c3c4s5
                                                )/(d7*c3s5_c4c5s3 - d4*c3);
            //dtheta7左臂与右臂一样，但是过渡式子不一样
             dtheta7=(dax*(c1*s3s5_c3c4c5 - s1*dist7_1) + 
                                                day*(-s0*dist7_2 + c0*(s1*s3s5_c3c4c5 + c1*dist7_1)) + 
                                                daz*(s0*(s1*s3s5_c3c4c5 + c1*dist7_1) + c0*dist7_2) + 
                                                dtheta2*(ax*s1*dist7_2 - ay*(s0*dist7_1 + c0*c1*dist7_2) + az*(c0*dist7_1 - s0*c1*dist7_2)) + //
                                                dtheta3*(ax*(c1*c3s5_c4c5s3 - s1s2*s3s5_c3c4c5) + ay*(c0s1*c3s5_c4c5s3 + s0c2_c0c1s2*s3s5_c3c4c5) - az*(c0c2_s0c1s2*s3s5_c3c4c5 - s0s1*c3s5_c4c5s3)) +
                                                dtheta5*(ax*(c1*c5s3_c3c4s5 + s1*dist7_4) - ay*(c0*c1*dist7_4 - c0s1*c5s3_c3c4s5 + s0*dist7_3) + az*(c0*dist7_3 - s0*c1*dist7_4 + s0*s1*c5s3_c3c4s5)
                                                ))/-c7;

             dtheta6=-(dtheta3*s4*(ax*(c1*s3 - c3*s1s2) +  ay*(c0*s1s3_c1c3s2 + s0c2*c3)) -  
                                dax*(-s1*c2c4_s2s3s4 + c1*c3*s4) + 
                                day*(-c0*(c1*c2c4_s2s3s4 + c3*s1*s4) + s0*c4s2_c2s3s4) - 
                                daz*(s0*(c1*c2c4_s2s3s4 + c3*s1*s4) + c0*c4s2_c2s3s4) + 
                                dtheta2*(-ax*(s1*c4s2_c2s3s4) + ay*(s0*c2c4_s2s3s4 + c0*c1*c4s2_c2s3s4)) - az*(c0*c2c4_s2s3s4 - s0*c4s2_c2s3s4) +
                                dtheta7*c6*s7)/(c7*s6);

             dtheta8=(dtheta5*(nx*(c1*c5s3_c3c4s5 + s1*dist7_4) - 
                                                                    ny*(c0*c1*dist7_4 - c0s1*c5s3_c3c4s5 + s0*dist7_3) + 
                                                                    nz*(c0*dist7_3 - s0*c1*dist7_4 + s0*s1*c5s3_c3c4s5)) +                                        
                                                dtheta3*(nx*(c1*c3s5_c4c5s3 + s1s2*s3s5_c3c4c5) + 
                                                                    ny*(c0s1*c3s5_c4c5s3 - s0c2_c0c1s2*s3s5_c3c4c5) +
                                                                    nz*(c0c2_s0c1s2*s3s5_c3c4c5 + s0s1*c3s5_c4c5s3)) +
                                                dtheta2*(nx*s1*dist7_2 - 
                                                                    ny*(s0*dist7_1 + c0*c1*dist7_2) + 
                                                                    nz*(c0*dist7_1 - s0*c1*dist7_2)) -
                                                dtheta7*c8*s7)/(c7*s8);
    // dtheta2_dist1L=(a1c2 + pxc2s1 - pz*c0s2 + pys0s2 - pyc0c1c2 - pzs0c1c2 - a8nxc2s1 + a8nzc0s2 - a8nys0s2 + a8*ny*c0c1c2 + a8nzs0c1c2);
// dtheta2_dist2L=(a1*s2 + pzc0c2 - pys0c2 + pxs1s2 - pyc0c1s2 - pzs0c1s2 - a8nzc0c2 + a8nys0c2 - a8nxs1s2 + a8nyc0c1s2 + a8nzs0c1s2);
// dtheta2_dist3L=(d2 - pxc1 + a8nxc1 - pyc0s1 - pzs0s1 + a8nyc0s1 + a8nzs0s1);
// dtheta2_dist4L=(pow(dtheta2_dist1L,2) + pow(dtheta2_dist2L,2) - pow(d4,2) - pow(d7,2) + pow(dtheta2_dist3L,2))/pow2d4d7;

// dtheta2=(powd7s4*(
//     dnz*((2*dist2_5*dist2_1)/powd7s4 + (2*dist2_5*dist2_1 - 2*(a8*c0c2 - a8*s0c1s2)*dist2_2 + 2*a8*s0s1*dist2_3)*dist2_4) - 
//     dpy*((2*(s0c2 + c0c1s2)*dist2_2 - 2*s0s2_c0c1c2*dist2_1 + 2*c0s1*dist2_3)*dist2_4 - (2*s0s2_c0c1c2*dist2_1)/powd7s4) + 
//     dpx*((2*s1s2*dist2_2 - 2*c1*dist2_3 + 2*c2s1*dist2_1)*dist2_4 + (2*c2s1*dist2_1)/powd7s4) - 
//     dpz*((2*c0s2_s0c1c2*dist2_1 - 2*(c0c2 - s0c1s2)*dist2_2 + 2*s0s1*dist2_3)*dist2_4 + (2*c0s2_s0c1c2*dist2_1)/powd7s4) - 
//     dnx*((2*a8*c2s1*dist2_1 - 2*a8*c1*dist2_3 + 2*a8*s1s2*dist2_2)*dist2_4 + (2*a8*c2s1*dist2_1)/powd7s4) + 
//     dny*((2*(a8*s0c2 + a8*c0c1s2)*dist2_2 - 2*dist2_6*dist2_1 + 2*a8*c0s1*dist2_3)*dist2_4 - (2*dist2_6*dist2_1)/powd7s4)))/
//     (2*dist2_1*dist2_2);

// double dtheta5=(dpz*c0s2_s0c1c2 - 
// dpy*s0s2_c0c1c2 + 
// dtheta2*dist2_2 + 
// dny*dist2_6 - 
// dnz*dist2_5 - 
// dpx*c2s1 + 
// a8*dnx*c2s1)/(d7*s4*s5)

// dtheta3=-(dpx*c1 + 
// dpy*c0s1 + 
// dpz*s0s1 + 
// d7*dtheta5*c5s3_c3c4s5 - 
// a8*dnx*c1 - 
// a8*dny*c0s1 - 
// a8*dnz*s0s1)/
// (d7*c3s5_c4c5s3 - d4*c3)

// dtheta7=(day*(c2*c3*s0*s5 + c0s1*s3*s5 - c5*s0s2*s4 + c0c1c2*c5*s4 - c0*c3*c4*c5*s1 + c0*c1*c3*s2*s5 + c2*c4*c5*s0*s3 + c0*c1*c4*c5*s2*s3) + 
// dtheta2*(az*c0c2*c5*s4 - ax*c2*c3*s1*s5 - ay*c2*c5*s0*s4 + az*c0*c3*s2*s5 + ax*c5*s1s2*s4 - ay*c3*s0s2*s5 + ay*c0c1c2*c3*s5 - ax*c2*c4*c5*s1*s3 - ay*c0*c1*c5*s2*s4 + az*c1*c2*c3*s0*s5 + az*c0*c4*c5*s2*s3 - ay*c4*c5*s0s2*s3 - az*c1*c5*s0s2*s4 + ay*c0c1c2*c4*c5*s3 + az*c1*c2*c4*c5*s0*s3) + 
// daz*(c0*c5*s2*s4 - c0c2*c3*s5 + s0s1*s3*s5 - c0c2*c4*c5*s3 + c1*c2*c5*s0*s4 - c3*c4*c5*s0s1 + c1*c3*s0s2*s5 + c1*c4*c5*s0s2*s3) - 
// dax*(c1*c3*c4*c5 - c1*s3*s5 + c2*c5*s1*s4 + c3*s1s2*s5 + c4*c5*s1s2*s3) + 
// dtheta3*(ax*c1*c3*s5 + ax*c1*c4*c5*s3 + ay*c0*c3*s1*s5 + az*c0c2*s3*s5 - ay*s0c2*s3*s5 + az*c3*s0s1*s5 + ax*s1s2*s3*s5 + ay*c2*c3*c4*c5*s0 - ax*c3*c4*c5*s1s2 + ay*c0*c4*c5*s1*s3 - ay*c0c1s2*s3*s5 + az*c4*c5*s0s1*s3 - az*s0c1s2*s3*s5 - az*c0c2*c3*c4*c5 + ay*c0*c1*c3*c4*c5*s2 + az*c1*c3*c4*c5*s0s2) + 
// dtheta5*(ax*c1*c5*s3 - az*c0c2*c3*c5 + ax*c1*c3*c4*s5 + ay*c2*c3*c5*s0 - ax*c3*c5*s1s2 + ay*c0*c5*s1*s3 + ax*c2s1*s4*s5 + az*c5*s0s1*s3 - az*c0s2*s4*s5 + ay*s0s2*s4*s5 + ay*c0*c1*c3*c5*s2 - ay*c0c1c2*s4*s5 + ay*c0*c3*c4*s1*s5 + az*c1*c3*c5*s0s2 + az*c0c2*c4*s3*s5 - ay*c2*c4*s0*s3*s5 - az*s0c1c2*s4*s5 + az*c3*c4*s0s1*s5 + ax*c4*s1s2*s3*s5 - az*c1*c4*s0s2*s3*s5 - ay*c0*c1*c4*s2*s3*s5))/c7

// dtheta6=-(dtheta3*(ax*c1*s3*s4 - az*c0c2*c3*s4 + ay*c2*c3*s0*s4 - ax*c3*s1s2*s4 + ay*c0s1*s3*s4 + az*s0s1*s3*s4 + ay*c0*c1*c3*s2*s4 + az*c1*c3*s0s2*s4) - 
// dax*(c1*c3*s4 - c2*c4*s1 + s1s2*s3*s4) + 
// day*(c4*s0s2 - c0c1c2*c4 - c0*c3*s1*s4 + s0c2*s3*s4 + c0c1s2*s3*s4) - 
// daz*(c0*c4*s2 + c1*c2*c4*s0 + c0c2*s3*s4 + c3*s0s1*s4 - s0c1s2*s3*s4) + 
// dtheta2*(ay*c2*c4*s0 - az*c0c2*c4 - ax*c4*s1s2 + ay*c0*c1*c4*s2 + az*c1*c4*s0s2 - ax*c2s1*s3*s4 + az*c0s2*s3*s4 - ay*s0s2*s3*s4 + ay*c0c1c2*s3*s4 + az*s0c1c2*s3*s4) + 
// dtheta7*c6*s7)/(c7*s6)
// dtheta8=(dtheta5*(nx*c1*c5*s3 - nz*c0c2*c3*c5 + nx*c1*c3*c4*s5 + ny*c2*c3*c5*s0 - nx*c3*c5*s1s2 + ny*c0*c5*s1*s3 + nx*c2s1*s4*s5 + nz*c5*s0s1*s3 - nz*c0s2*s4*s5 + ny*s0s2*s4*s5 + ny*c0*c1*c3*c5*s2 - ny*c0c1c2*s4*s5 + ny*c0*c3*c4*s1*s5 + nz*c1*c3*c5*s0s2 + nz*c0c2*c4*s3*s5 - ny*c2*c4*s0*s3*s5 - nz*s0c1c2*s4*s5 + nz*c3*c4*s0s1*s5 + nx*c4*s1s2*s3*s5 - ny*c0*c1*c4*s2*s3*s5 - nz*c1*c4*s0s2*s3*s5) + 
// dtheta3*(nx*c1*c3*s5 + nx*c1*c4*c5*s3 + ny*c0*c3*s1*s5 + nz*c0c2*s3*s5 - ny*s0c2*s3*s5 + nz*c3*s0s1*s5 + nx*s1s2*s3*s5 - nz*c0c2*c3*c4*c5 + ny*c2*c3*c4*c5*s0 - nx*c3*c4*c5*s1s2 + ny*c0*c4*c5*s1*s3 - ny*c0c1s2*s3*s5 + nz*c4*c5*s0s1*s3 - nz*s0c1s2*s3*s5 + ny*c0*c1*c3*c4*c5*s2 + nz*c1*c3*c4*c5*s0s2) + 
// dtheta2*(nz*c0c2*c5*s4 - nx*c2*c3*s1*s5 - ny*c2*c5*s0*s4 + nz*c0*c3*s2*s5 + nx*c5*s1s2*s4 - ny*c3*s0s2*s5 + ny*c0c1c2*c3*s5 - nx*c2*c4*c5*s1*s3 - ny*c0*c1*c5*s2*s4 + nz*c1*c2*c3*s0*s5 + nz*c0*c4*c5*s2*s3 - ny*c4*c5*s0s2*s3 - nz*c1*c5*s0s2*s4 + ny*c0c1c2*c4*c5*s3 + nz*c1*c2*c4*c5*s0*s3) - 
// dtheta7*c8*s7)/(c7*s8)
    }

//右臂
    else
{
// float diff_dnz=(2*dist2_5*dist2_1)/powd7s4 -(2*a8*c0c2_s0c1s2*dist2_2 - 2*dist2_5*dist2_1 + 2*a8*s0s1*dist2_3)*dist2_4;
// float diff_dpy=(2*s0s2_c0c1c2*dist2_1)/powd7s4 + (2*s0s2_c0c1c2*dist2_1 - 2*s0c2_c0c1s2*dist2_2 + 2*c0s1*dist2_3)*dist2_4;
// float diff_dpx=(2*c1*dist2_3 + 2*s1s2*dist2_2 + 2*c2s1*dist2_1)*dist2_4 + (2*c2s1*dist2_1)/powd7s4;
// float diff_dpz=(2*c0c2_s0c1s2*dist2_2 - 2*c0s2_s0c1c2*dist2_1 + 2*s0s1*dist2_3)*dist2_4 - (2*c0s2_s0c1c2*dist2_1)/powd7s4;
// float diff_dnx=(2*a8*c1*dist2_3 + 2*a8*c2s1*dist2_1 + 2*a8*s1s2*dist2_2)*dist2_4 + (2*a8*c2s1*dist2_1)/powd7s4;
// float diff_dny=(2*dist2_6*dist2_1 - 2*a8*s0c2_c0c1s2*dist2_2 + 2*a8*c0s1*dist2_3)*dist2_4 + (2*dist2_6*dist2_1)/powd7s4;
        dtheta2=-(powd7s4*(
                dnz*((2*dist2_5*dist2_1)/powd7s4 + (2*dist2_5*dist2_1 - 2*a8*c0c2_s0c1s2*dist2_2 - 2*a8*s0s1*dist2_3)*dist2_4) + 
                dpy*((2*s0s2_c0c1c2*dist2_1)/powd7s4 + (2*s0s2_c0c1c2*dist2_1 - 2*s0c2_c0c1s2*dist2_2 + 2*c0s1*dist2_3)*dist2_4) + 
                dpx*((2*c1*dist2_3 + 2*s1s2*dist2_2 + 2*c2s1*dist2_1)*dist2_4 + (2*c2s1*dist2_1)/powd7s4) + 
                dpz*((2*c0c2_s0c1s2*dist2_2 - 2*c0s2_s0c1c2*dist2_1 + 2*s0s1*dist2_3)*dist2_4 - (2*c0s2_s0c1c2*dist2_1)/powd7s4) - 
                dnx*((2*a8*c1*dist2_3 + 2*a8*c2s1*dist2_1 + 2*a8*s1s2*dist2_2)*dist2_4 + (2*a8*c2s1*dist2_1)/powd7s4) - 
                dny*((2*dist2_6*dist2_1 - 2*a8*s0c2_c0c1s2*dist2_2 + 2*a8*c0s1*dist2_3)*dist2_4 + (2*dist2_6*dist2_1)/powd7s4)
                ))/(2*dist2_1*dist2_2);       

        dtheta5=-(dpx*c2s1 + 
                                            dpy*s0s2_c0c1c2 - 
                                            dpz*c0s2_s0c1c2  - 
                                            dnx*a8*c2s1 - 
                                            dny*dist2_6 + 
                                            dnz*dist2_5   + 
                                            dtheta2*dist2_2
                                            )/(d7*s4*s5);

        dtheta3=-(dpx*c1 + 
                                            dpy*c0s1 + 
                                            dpz*s0s1 - 
                                            dnx*a8*c1 - 
                                            dny*a8*c0s1 - 
                                            dnz*a8*s0s1 + 
                                            dtheta5*d7*c5s3_c3c4s5 
                                            )/(d7*c3s5_c4c5s3 + d4*c3);

        dtheta7=(dax*(c1*s3s5_c3c4c5 - s1*dist7_1) +
                                            day*(-s0*dist7_2 + c0*(s1*s3s5_c3c4c5 + c1*dist7_1)) +
                                            daz*(s0*(s1*s3s5_c3c4c5 + c1*dist7_1) + c0*dist7_2) +
                                            dtheta2*(ax*s1*dist7_2 - ay*(s0*dist7_1 + c0*c1*dist7_2) + az*(c0*dist7_1 - s0*c1*dist7_2)) +
                                            dtheta3*(ax*(c1*c3s5_c4c5s3 - s1s2*s3s5_c3c4c5) + ay*(c0s1*c3s5_c4c5s3 + s0c2_c0c1s2*s3s5_c3c4c5) - az*(c0c2_s0c1s2*s3s5_c3c4c5 - s0s1*c3s5_c4c5s3)) +
                                            dtheta5*(ax*(c1*c5s3_c3c4s5 + s1*dist7_4) - ay*(c0*c1*dist7_4 - c0s1*c5s3_c3c4s5 + s0*dist7_3) + az*(c0*dist7_3 - s0*c1*dist7_4 + s0*s1*c5s3_c3c4s5))
                                            )/-c7;

        dtheta6=-(day*(c0*(c1*c2c4_s2s3s4 + c3*s1*s4) - s0*c4s2_c2s3s4) - 
                            dtheta3*s4*(ax*(c1*s3 +  c3*s1s2) + ay*(c0*s1s3_c1c3s2 - s0c2*c3) + az*(c0c2*c3 + s0*s1s3_c1c3s2) - 
                            dax*(s1*c2c4_s2s3s4 - c1*c3*s4) + 
                            daz*(s0*(c1*c2c4_s2s3s4 + c3*s1*s4) + c0*c4s2_c2s3s4) + 
                            dtheta2*(ax*(s1*c4s2_c2s3s4) - ay*(s0*c2c4_s2s3s4 + c0*c1*c4s2_c2s3s4) + az*(c0*c2c4_s2s3s4 - s0*c4s2_c2s3s4)) + 
                            dtheta7*c6*s7))/(c7*s6);

        dtheta8=(dtheta5*(nx*(c1*c5s3_c3c4s5 + s1*dist7_4) - 
                                                ny*(c0*c1*dist7_4 + c0s1*c5s3_c3c4s5 + s0*dist7_3) + 
                                                nz*(c0*dist7_3 - s0*c1*dist7_4 + s0*s1*c5s3_c3c4s5)) + 
                            dtheta3*(nx*(c1*c3s5_c4c5s3 - s1s2*s3s5_c3c4c5) + 
                                                ny*(c0s1*c3s5_c4c5s3 + s0c2_c0c1s2*s3s5_c3c4c5) + 
                                                - nz*(c0c2_s0c1s2*s3s5_c3c4c5 - s0s1*c3s5_c4c5s3)) + //
                            dtheta2*(nx*s1*dist7_2 - 
                                                ny*(s0*dist7_1 + c0*c1*dist7_2) + 
                                                nz*(c0*dist7_1 - s0*c1*dist7_2)) - 
                            dtheta7*c8*s7)/(c7*s8);

}
    target_dtheta << dtheta2,dtheta3,dtheta5,dtheta6,dtheta7,dtheta8;
    return target_dtheta;
}

//Matrix<double,6,1> teleope_homo_hetero::slave_shoulder_pos_FK();

//踏板有5个按键# #          H   J
//                           # # #      B    N   M
/*1.上电开急停
2.启动 长按N    外骨骼运动至初始位置
3.佩戴 按B         外骨骼手臂伸直    按M返回上一步
4.佩戴完毕按B          外骨骼回初始位置  按M返回上一步
5.按N主从对齐 3s后主从对齐完毕后，上肢放松，按N传感器归0
6.传感器归0完毕 从爪动一动，
7.按M（再按B或M选择左右单臂）再按N 同构操作/按B(再按B或M选择左右单臂 )(再长按B外骨骼外骨骼回异构舒适位置)再按N进行
8.切换按N暂停，回到7
9.暂停之后 长按N结束 外骨骼运动至初始位置 机械臂也回到初始位置（再长按N又回到2）
10.同时按HJ外骨骼与机械臂均回到关闭位置，断电拍急停
*/

// int main(int argc, char *argv[])
// {
//     ros::init(argc, argv, "teleope");

// }