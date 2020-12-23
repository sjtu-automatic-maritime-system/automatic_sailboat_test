//
// File: ert_main.cpp
//
// Code generated for Simulink model 'sailboat_ap_ok'.
//
// Model version                  : 1.221
// Simulink Coder version         : 8.6 (R2014a) 27-Dec-2013
// C/C++ source code generated on : Tue Sep 26 11:36:58 2017
//
// Target selection: ert.tlc
// Embedded hardware selection: Intel->x86-64 (Linux 64)
// Code generation objectives:
//    1. Execution efficiency
//    2. RAM efficiency
// Validation result: Not run
//
#include <stddef.h>
#include <stdio.h>                     // This ert_main.c example uses printf/fflush
#include "spare_function/sailboat_ap_ok.h"            // Model's header file
#include "rtwtypes.h"

#include "ros/ros.h"
#include "spare_function/spare_function_Config.h"
#include "spare_function/spare_function_out.h"
#include "spare_function/spare_function_para.h"

#include "sailboat_message/Sensor_msg.h"
#include "sailboat_message/Mach_msg.h"
#include <dynamic_reconfigure/server.h>
#include <spare_function/sailboat_ap_ok.h>
using namespace std;
#include <math.h>
#include <unistd.h>

static sailboat_ap_okModelClass rtObj; // Instance of model class
double YawAngle,WinAngle,PosisionX,PosisionY,TargetX,TargetY,RudderAngle,SailAngle,TargetAngle,u,v,w,AbsWinAngle,awa,aws,d=0;
const double Pi=3.14159265;
const double HowClose=1;            //用于判定是否到达目标点

int pcCtrl = 0;


//
// Associating rt_OneStep with a real-time clock or interrupt service routine
// is what makes the generated code "real-time".  The function rt_OneStep is
// always associated with the base rate of the model.  Subrates are managed
// by the base rate from inside the generated code.  Enabling/disabling
// interrupts and floating point context switches are target specific.  This
// example code indicates where these should take place relative to executing
// the generated code step function.  Overrun behavior should be tailored to
// your application needs.  This example simply sets an error status in the
// real-time model and returns from rt_OneStep.
//
void rt_OneStep(void)
{
  static boolean_T OverrunFlag = 0;

  // Disable interrupts here

  // Check for overrun
  if (OverrunFlag) {
    rtmSetErrorStatus(rtObj.getRTM(), "Overrun");
    return;
  }

  OverrunFlag = true;

  // Save FPU context here (if necessary)
  // Re-enable timer or interrupt here
  // Set model inputs here

  // Step the model
  rtObj.step();

  // Get model outputs here

  // Indicate task complete
  OverrunFlag = false;

  // Disable interrupts here
  // Restore FPU context here (if necessary)
  // Enable interrupts here
}

void getInput(const sailboat_message::Sensor_msg::ConstPtr msg) {
  YawAngle = msg->Yaw;
  awa=msg->AWA;
  u=msg->ux;
  v=msg->vy;
  aws=msg->AWS;             //绝对风向角，风去向与坐标系X轴夹角
}

void getPara(spare_function::spare_function_Config &config){
  rtObj.rtP.oyaw    = config.oyaw;
  rtObj.rtP.rudderP = config.rudderP;
  rtObj.rtP.rudderI = config.rudderI;
  rtObj.rtP.rudderD = config.rudderD;
  rtObj.rtP.sailP   = config.sailP;
  rtObj.rtP.sailI   = config.sailI;
  rtObj.rtP.sailD   = config.sailD;
}

double Deviation(double a,double b)    //计算两个矢量的方向，其中a与b都是角度，-180到180之间，输出c是一个b以a为参考坐标轴的-180到180之间的角度
{
    double c=a-b;                      //a-b是一个-360到360之间的一个数，烦死了，想了好久
    if(c>Pi) c=c-2*Pi;
    if(c<-Pi) c=c+2*Pi;
    return c;
}
double CalAbusWA()
{
    double a=Pi+YawAngle-awa;
    double o;
    double uw,vw,v1;
    uw=u*cos(YawAngle)-v*sin(YawAngle)+aws*cos(a);
    vw=v*cos(YawAngle)+u*sin(YawAngle)+aws*sin(a);
    v1=sqrt(uw*uw+vw*vw);
    o=asin(vw/v1);
    if(uw<0)
    {
        o=Pi-o;
    }
    if(o>Pi) o=o-2*Pi;
    if (o<-Pi) o=o+2*Pi;
    return o;

}

double Distance(double x1,double y1,double x2,double y2)
{
    double d=sqrt((x2-x1)*(x2-x1)+(y2-y1)*(y2-y1));
    return d;
}
/*double CalTargetAngle(double x1,double y1,double x2,double y2)//计算目标角。2为目标，1为目前位置
{
    double x=x2-x1,double y=y2-y1;//(x,y)为目标向量，船应沿此方向前进。
    double a = atan2(y,x);//计算目标角绝对值，将问题转化为航向保持。atan2以逆时针为正，返回值在-pi~pi间。
    return -a;//返回值逆时针为正，与设定相反，所以返回-a。
}*/

void Normal()                      //正常风向情况下，船向目标方向转,偏离越多转角越大，最大舵角30
{
    double T=Deviation(YawAngle,TargetAngle);
    RudderAngle=T/6;               //实船待修改

}

void AgainstWind(sailboat_message::Mach_msg &msg)
{
    double T=Deviation(YawAngle,TargetAngle);
    double W=Deviation(YawAngle,AbsWinAngle);
    if(sqrt(u*u+v*v)>0.5||abs(T)>Pi/12)  //这个最低船速在实船试验中需要更改
    {
        Normal();
        msg.rudder = RudderAngle;
        msg.sail   = 0;
        msg.timestamp = ros::Time::now().toSec();
        msg.motor = 57;
        msg.PCCtrl = pcCtrl;
        return;                         //如果船速还比较大或者仍然大角度偏离目标就继续按照正常行驶，转向目标，但是由于逆风所以帆角收紧
    }
    if(W>=0)                      //这里的WinAngle在getOutMachPut函数中已经赋值，全局变量可以直接使用
    {
        RudderAngle=Pi/6;
        SailAngle=Pi/9;
    }
    else
    {
        RudderAngle=-Pi/6;
        SailAngle=Pi/9;
    }
    msg.rudder = RudderAngle;
    msg.sail   = SailAngle;
    sleep(2);                           //逆风且朝向目标的情况下，执行逆风动作
    msg.motor = 57;
    msg.rudder = 0-RudderAngle;
    msg.sail   = SailAngle;
    msg.PCCtrl = pcCtrl;
    sleep(2);
    return;
}

void getOutMachPut(sailboat_message::Mach_msg &msg) {

    TargetAngle=rtObj.rtP.oyaw*Pi/180;//返回的值是角度制
    AbsWinAngle=CalAbusWA();
    /*
    switch(d<=HowClose)
    {
        case 1: SailAngle=0;
        d=Distance(PosisionX,PosisionY,TargetX,TargetY);
                TargetAngle=CalTargetAngle(PosisionX,PosisionY,TargetX,TargetY);//判断到达第一个目标点后，计算目标角，并调整targetangle
               Normal();
               msg.timestamp = ros::Time::now().toSec();
               msg.motor = 0;
               msg.rudder = RudderAngle;
               msg.sail   = SailAngle;
               msg.PCCtrl = pcCtrl;
               return;
        case 0:d=Distance(PosisionX,PosisionY,TargetX,TargetY);
                break;
    }*/
    //case0和1里面都有d是为了保证初始情况下能自动判断targetangle
    //令d=0，则在程序运行初始情况下case1，得出targetangle。
    WinAngle=Deviation(YawAngle,AbsWinAngle);  //计算相对风向角
                                               //建立起映射关系，相当于把坐标轴X移到船首方向时的绝对风向角，-180到180之间，吹向船首为0度
                                               //WinAngle为全局变量，这里计算赋值过后，其他函数也可以用，比如逆风情况的AgainstWind函数中
    switch(WinAngle>0)
    {
    case 1:                                    //相对风角>0，右舷来风,0-135为正常风向
        switch(WinAngle<(Pi*0.75))
        {
            case 1: SailAngle=0.667*(Pi*0.75-WinAngle); //帆角根据风向角从0-90变化,往左开帆为负
                    Normal();
                    break;
            case 0: AgainstWind(msg);
                    return;
        }
        break;
    case 0:                                    //相对风角<0，左舷来风，-135到0为正常风向，往右开帆为正
        switch(WinAngle>(-Pi*0.75))
        {
            case 1: SailAngle=0.667*(Pi*0.75+WinAngle);
                    Normal();
                    break;
            case 0: AgainstWind(msg);
                    return;
        }
        break;
    }

  msg.timestamp = ros::Time::now().toSec();
  msg.motor = 57;
  msg.rudder = RudderAngle;
  msg.sail   = SailAngle;
  msg.PCCtrl = pcCtrl;
}

void getOutput(spare_function::spare_function_out &msg) {

  msg.rudder = RudderAngle;
  msg.sail = SailAngle;
}

void getOutParaPut(spare_function::spare_function_para &msg) {
  msg.oyaw   = rtObj.rtP.oyaw   ;
  msg.rudderP= rtObj.rtP.rudderP;
  msg.rudderI= rtObj.rtP.rudderI;
  msg.rudderD= rtObj.rtP.rudderD;
  msg.sailP  = rtObj.rtP.sailP  ;
  msg.sailI  = rtObj.rtP.sailI  ;
  msg.sailD  = rtObj.rtP.sailD  ;
}

void cfgCallback(spare_function::spare_function_Config &config, uint32_t level) {
//ROS_INFO("Reconfigure Request: %f %f %f", config.Kp, config.Ki, config.Kd);

  if (config.PC_Ctrl == true)
    pcCtrl = 1;
  else
    pcCtrl = 0;
  getPara(config);
}

void sensorCallback(const sailboat_message::Sensor_msg::ConstPtr msg) {
  getInput(msg);
}

//
// The example "main" function illustrates what is required by your
// application code to initialize, execute, and terminate the generated code.
// Attaching rt_OneStep to a real-time clock is target specific.  This example
// illustates how you do this relative to initializing the model.
//
int_T main(int_T argc, char **argv)
{
  // Unused arguments
  (void)(argc);
  (void)(argv);

  // Initialize model
  rtObj.initialize();
  //double *RTargetAngle= new double;

  ros::init(argc, argv, "spare");
  ros::NodeHandle nh;
  ros::Subscriber sub;
  ros::Publisher spare_function_pub;
  ros::Publisher spare_function_para_pub;

  ros::Publisher mach_pub;
  spare_function_pub = nh.advertise<spare_function::spare_function_out>("spare_function_out", 2);
  spare_function_para_pub = nh.advertise<spare_function::spare_function_para>("spare_function_para", 2);
  mach_pub = nh.advertise<sailboat_message::Mach_msg>("mach", 2);

  dynamic_reconfigure::Server<spare_function::spare_function_Config> server;
  dynamic_reconfigure::Server<spare_function::spare_function_Config>::CallbackType f;

  f = boost::bind(&cfgCallback, _1, _2);
  server.setCallback(f);

//    dynamic_reconfigure::Server<scanning::pcCtrl_Config> server2;
//    dynamic_reconfigure::Server<scanning::pcCtrl_Config>::CallbackType f2;
//
//    f2 = boost::bind(&PcCtrlCfgcallback, _1, _2);
//    server2.setCallback(f2);

  sub = nh.subscribe("sensor", 100, sensorCallback);

  ros::Rate loop_rate(10);
  while (ros::ok()) {
    ros::spinOnce();
    rt_OneStep();
    spare_function::spare_function_out msg;
    sailboat_message::Mach_msg msgMach;
    spare_function::spare_function_para msgPara;

    getOutMachPut(msgMach);
    getOutput(msg);
    getOutParaPut(msgPara);

    spare_function_pub.publish(msg);
    spare_function_para_pub.publish(msgPara);
    mach_pub.publish(msgMach);

    loop_rate.sleep();
  }
  // Attach rt_OneStep to a timer or interrupt service routine with
  //  period 0.1 seconds (the model's base sample time) here.  The
  //  call syntax for rt_OneStep is
  //
  //   rt_OneStep();

  printf("Warning: The simulation will run forever. "
         "Generated ERT main won't simulate model step behavior. "
         "To change this behavior select the 'MAT-file logging' option.\n");
  fflush((NULL));
  while (rtmGetErrorStatus(rtObj.getRTM()) == (NULL)) {
    //  Perform other application tasks here
  }

  // Disable rt_OneStep() here

  // Terminate model
  rtObj.terminate();
  //delete RTargetAngle;
  return 0;
}

//
// File trailer for generated code.
//
// [EOF]
//
