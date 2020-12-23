# -*- coding:UTF-8 -*-
# import sys
# sys.path.append("../..")
import numpy as np
from numpy import sin,cos,pi,ceil,tan
# from scipy.linalg import inv

from msgdev import MsgDevice, PeriodTimer
# import matplotlib.pyplot as plt
import time
from collections import deque
dt=0.1


class EKF:
    def __init__(self,use_dynamic_model=True):
        self.x=np.zeros(6)
        self.P=10*np.eye(6)
        self.Q=np.array([[0.01**2,0,0,0,0,0],
                [0,0.01**2,0,0,0,0],
                [0,0,0.05**2,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0],
                [0,0,0,0,0,0]])
        self.R=np.array([[0.0554**2,0,0,0,0,0],
                [0,0.0554**2,0,0,0,0],
                [0,0,0.0045**2,0,0,0],
                [0,0,0,0.2**2,0,0],
                [0,0,0,0,0.2**2,0],
                [0,0,0,0,0,0.1**2]])
        self.use_dynamic_model=use_dynamic_model

    def update(self,y_ob,n1,n2):
        u,v,r,x,y,yaw=self.x
        F=np.array([[1,0,0,0,0,0],
                    [0,1,0,0,0,0],
                    [0,0,1,0,0,0],
                    [cos(yaw)*dt,-sin(yaw)*dt,0,1,0,(-u*sin(yaw)-v*cos(yaw))*dt],
                    [sin(yaw)*dt,cos(yaw)*dt,0,0,1,(u*cos(yaw)-v*sin(yaw))*dt],
                    [0,0,dt,0,0,1]])
        H=np.array([[cos(yaw),-sin(yaw),0,0,0,-u*sin(yaw)-v*cos(yaw)],
                    [sin(yaw),cos(yaw),0,0,0,u*cos(yaw)-v*sin(yaw)],
                    [0,0,1,0,0,0],
                    [0,0,0,1,0,0],
                    [0,0,0,0,1,0],
                    [0,0,0,0,0,1]])
        xp=np.array(self.transform(self.x,n1,n2))
        Pp=np.dot(np.dot(F,self.P),F.T)+self.Q
        tmp=np.array(np.matrix(np.dot(np.dot(H,Pp),H.T)+self.R).I)
        k=np.dot(np.dot(Pp,H.T),tmp)
        y_=np.array(self.observe(xp))
        res=y_ob-y_
        res[-1]=self.yawRange(res[-1])
        self.x=xp+np.dot(k,res)
        self.x[-1] = self.yawRange(self.x[-1])
        self.P=np.dot(np.eye(6)-np.dot(k,H),Pp)

    def transform(self,s, n1, n2):
        def acceleration(u, v, r, n1, n2):
            ax = (58.0 * r * v - 6.7 * u * abs(u) + 15.9 * r ** 2 + 0.01205 *
                  (n1 * abs(n1) + n2 * abs(n2)) - 0.0644 * (
                          u * (abs(n1) + abs(n2)) + 0.45 * r * (abs(n1) - abs(n2)))) / 33.3
            ay = (-33.3 * r * u - 29.5 * v + 11.8 * r) / 58
            ar = (-0.17 * v - 2.74 * r - 4.78 * r * abs(r) + 0.45 *
                  (0.01205 * (n1 * abs(n1) - n2 * abs(n2)) - 0.0644 * (
                          u * (abs(n1) - abs(n2)) + 0.45 * r * (abs(n1) + abs(n2))))) / 6.1
            return ax, ay, ar
        u, v, r, x, y, yaw = s
        if self.use_dynamic_model:
            ax, ay, ar = acceleration(u, v, r, n1/60, n2/60)
        else:
            ax,ay,ar=0,0,0
        u1=u+ax*dt
        v1=v+ay*dt
        r1=r+ar*dt
        x1 = x + (u * cos(yaw) - v * sin(yaw)) * dt
        y1 = y + (u * sin(yaw) + v * cos(yaw)) * dt
        yaw1 = yaw + r * dt
        return [u1, v1, r1, x1, y1, yaw1]

    def observe(self,s):
        u, v, r, x, y, yaw = s
        uo=u * cos(yaw) - v * sin(yaw)
        vo=u * sin(yaw) + v * cos(yaw)
        return [uo,vo,r,x,y,yaw]

    def yawRange(self,x):
        if x > pi:
            x = x - 2 * pi
        elif x < -pi:
            x = x + 2 * pi
        return x


class Interface(object):
    def __init__(self,sub_addr,ahrs_port=None,gnss_port=None,motor_read_port=None,voltage_port=None,pub_port=None):
        self.dev=MsgDevice()
        self.dev.open()
        if ahrs_port:
            self.dev.sub_connect(sub_addr+':'+ahrs_port)
            self.dev.sub_add_url('ahrs.roll')
            self.dev.sub_add_url('ahrs.pitch')
            self.dev.sub_add_url('ahrs.yaw')
            self.dev.sub_add_url('ahrs.roll_speed')
            self.dev.sub_add_url('ahrs.pitch_speed')
            self.dev.sub_add_url('ahrs.yaw_speed')
            self.dev.sub_add_url('ahrs.acce_x')
            self.dev.sub_add_url('ahrs.acce_y')
            self.dev.sub_add_url('ahrs.acce_z')
        if gnss_port:
            self.dev.sub_connect(sub_addr+':'+gnss_port)
            self.dev.sub_add_url('gps.time')
            self.dev.sub_add_url('gps.posx')
            self.dev.sub_add_url('gps.posy')
            self.dev.sub_add_url('gps.posz')
            self.dev.sub_add_url('gps.stdx')
            self.dev.sub_add_url('gps.stdy')
            self.dev.sub_add_url('gps.stdz')
            self.dev.sub_add_url('gps.satn')
            self.dev.sub_add_url('gps.hspeed')
            self.dev.sub_add_url('gps.vspeed')
            self.dev.sub_add_url('gps.track')
        if motor_read_port:
            self.dev.sub_connect(sub_addr+':'+motor_read_port)
            self.dev.sub_add_url('left.Motor_SpeedCalc')
            self.dev.sub_add_url('right.Motor_SpeedCalc')
        if voltage_port:
            self.dev.sub_connect(sub_addr+':'+voltage_port)
            self.dev.sub_add_url('voltage')
        if pub_port:
            self.dev.pub_bind('tcp://0.0.0.0:'+pub_port)

    def receive(self,*args):
        data=[]
        for i in args:
            data.append(self.dev.sub_get1(i))
        return data

    def publish(self,dic):
        for key in dic:
            if isinstance(dic[key],list):
                self.dev.pub_set(key,dic[key])
            else:
                self.dev.pub_set1(key,dic[key])


def initialize():
    #USV150
    sub_addr = 'tcp://192.168.1.150'
    ahrs_port = '55005'
    gnss_port = '55004'
    motor_read_port = '55003'
    voltage_port = '55006'
    pub_port='55007'
    interface=Interface(sub_addr=sub_addr,ahrs_port=ahrs_port,gnss_port=gnss_port,motor_read_port=motor_read_port,voltage_port=voltage_port,pub_port=pub_port)
    ekf=EKF()
    return interface,ekf

def update(interface,ekf):
    data = interface.receive('ahrs.yaw', 'ahrs.yaw_speed', 'gps.posx', 'gps.posy', 'gps.hspeed', 'gps.track',
                             'left.Motor_SpeedCalc', 'right.Motor_SpeedCalc')
    yaw,r,x,y,hspeed,track,left,right=data
    print('origin left:{:.2f},right:{:.2f},hspeed:{:.2f},track:{:.2f},r:{:.2f},x:{:.2f},y:{:.2f},yaw:{:.2f}'.format(left,right,hspeed,track,r,x,y,yaw))
    left=-left
    yaw=yaw-5*pi/180
    uo=hspeed*cos(track)
    vo=hspeed*sin(track)
    ekf.update(np.array([uo,vo,r,x,y,yaw]),left,right)
    dic={'USV150.state':ekf.x.tolist(),'left.Motor_SpeedCalc':left,'right.Motor_SpeedCalc':right}
    interface.publish(dic)

    print("kalman left:{:.2f},right:{:.2f},u:{:.2f},v:{:.2f},r:{:.2f},x:{:.2f},y:{:.2f},yaw:{:.2f}".format(left, right, *ekf.x.tolist()))

if __name__=="__main__":
    try:
        interface,ekf=initialize()
        t=PeriodTimer(dt)
        t.start()
        while True:
            with t:
                update(interface,ekf)
    except (KeyboardInterrupt,Exception) as e:
        interface.dev.close()
        raise
    finally:
        pass



