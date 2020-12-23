//
// Created by hywel on 17-5-5.
//

#include "sensor_process/CSensorProcess.h"




CSensorProcess::CSensorProcess() {

    AhrsMsg = new double[10];
    WtstMsg = new double[11];
    SensorMsg = new double[11];

    Init();
}

CSensorProcess::~CSensorProcess() {
    delete [] AhrsMsg;
    delete [] WtstMsg;
    delete [] SensorMsg;
}

void CSensorProcess::Init() {
    sensor_pub = node.advertise<sailboat_message::Sensor_msg>("sensor2", 2);
    ahrs_sub = node.subscribe("ahrs", 2, &CSensorProcess::ahrsCallback,this);
    wtst_sub = node.subscribe("wtst", 2, &CSensorProcess::wtstCallback,this);
}

void CSensorProcess::ProcessMsg() {
    SensorMsg[1] = 0; //todo!
    SensorMsg[2] = 0; //todo!
    SensorMsg[3] = AhrsMsg[4];
    SensorMsg[4] = AhrsMsg[6];
    SensorMsg[5] = WtstMsg[4];
    SensorMsg[6] = WtstMsg[5];
    SensorMsg[7] = WtstMsg[6];
    SensorMsg[8] = WtstMsg[8];
    SensorMsg[9] = WtstMsg[9];
    SensorMsg[10] = WtstMsg[10];
}

double* CSensorProcess::GetSensorMsg() {
    ProcessMsg();
    return SensorMsg;
}


void CSensorProcess::ahrsCallback(const sailboat_message::Ahrs_msg::ConstPtr &msg) {
    ROS_INFO("ahrs_msg sub: [%f] [%f] [%f]", msg->roll,msg->pitch,msg->yaw);
    //AhrsMsg[0] = msg->timestamp;
    AhrsMsg[1] = msg->roll;
    AhrsMsg[2] = msg->pitch;
    AhrsMsg[3] = msg->yaw;
    AhrsMsg[4] = msg->gx;
    AhrsMsg[5] = msg->gy;
    AhrsMsg[6] = msg->gz;
    AhrsMsg[7] = msg->ax;
    AhrsMsg[8] = msg->ay;
    AhrsMsg[9] = msg->az;
}

void CSensorProcess::wtstCallback(const sailboat_message::WTST_msg::ConstPtr& msg)
{
    ROS_INFO("wtst_msg sub: [%f] [%f]", msg->PosX,msg->PosY);
    //WtstMsg[0] = msg->timestamp;
    WtstMsg[1] = msg->GPSIndicator;
    WtstMsg[2] = msg->Latitude;
    WtstMsg[3] = msg->Longitude;
    WtstMsg[4] = msg->PosX;
    WtstMsg[5] = msg->PosY;
    WtstMsg[6] = msg->Roll/57.3;
    WtstMsg[7] = msg->Pitch/57.3;
    WtstMsg[8] = msg->Yaw/57.3;
    WtstMsg[9] = msg->WindAngle/57.3;
    WtstMsg[10] = msg->WindSpeed*0.514;
    if (WtstMsg[9]>3.14)
        WtstMsg[9] = WtstMsg[9]-6.28;
}
