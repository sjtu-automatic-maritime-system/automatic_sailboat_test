#!/usr/bin/env python
# -*- coding: utf-8-*
import rospy

from sailboat_message.msg import Sensor_msg
from sailboat_message.msg import Mach_msg

from spare_function.cfg import spare_function_Config
from spare_function.msg import spare_function_out
from spare_function.msg import spare_function_para
from collections import deque
from collections import Counter

from dynamic_reconfigure.server import Server
import math
import numpy as np

# from numpy import sin,cos,pi,arctan2,sqrt,rad2deg


sensor_submsg = [0,0,0,0,0,0,0,0,0,0]
para_cfg = [0,0,0,0,0,0,0,0]


LRX = np.load('lrx.npy')
temp = deque(maxlen=10) #存储每次计算strategy_num   
strategy = deque(maxlen=10)
strategy.append(26)#存储输出strategy_num      
sa_out = deque(maxlen=10)
sa_out.append(0)#输出帆角        
u_numlist=deque(maxlen=10) 
u_numlist.append(0) #存储速度num
heading_error = deque(maxlen=100)
heading_error.append(0)
heading_error_old=deque(maxlen=1)
heading_error_old.append(0)
awa_list=deque(maxlen=2)
awa_list.append(0)
label=deque(maxlen=2)
label.append(0)
heading_limt=deque(maxlen=1)
heading_limt.append(0)
LABEL=0
label_tacking_time=0

# print(LRX.shape)
# print()

# 根据角度判断状态代码
def angle2num(angle):
    if angle > -np.pi*7/8 and angle <= -np.pi*5/8:
        num = 6
    elif angle > -np.pi*5/8 and angle <= -np.pi*3/8:
        num = 7
    elif angle > -np.pi*3/8 and angle <= -np.pi/8:
        num = 8
    elif angle > -np.pi/8 and angle <= np.pi/8:
        num = 1
    elif angle > np.pi/8 and angle <= np.pi*3/8:
        num = 2
    elif angle > np.pi*3/8 and angle <= np.pi*5/8:
        num = 3
    elif angle > np.pi*5/8 and angle <= np.pi*7/8:
        num = 4
    else:
        num = 5
    return num

# 根据u判断速度代码
def u2num(u,tws):
    limit_u=0.6
    if u > limit_u:
        u_num = 1
    else:
        u_num = 0
    return u_num

def goal_angle_func(ship_x, ship_y, heading, goal_x, goal_y):
    goal_angle = math.atan2(goal_y-ship_y, goal_x-ship_x)-heading
    goal_angle = angle_limit(goal_angle)
    goal_num=angle2num(goal_angle)
    return goal_angle, goal_num


def aw2tw(aws, awa, u, v, heading):
    vx = u*np.cos(heading)-v*np.sin(heading)
    vy = u*np.sin(heading)+v*np.cos(heading)
    twx = aws*math.cos(awa+heading+np.pi)+vx
    twy = aws*math.sin(awa+heading+np.pi)+vy
    tws = np.sqrt(twx*twx+twy*twy)
    if awa>0:
        twa = angle_limit(math.atan2(twy, twx)+np.pi)
    else:
        twa = angle_limit(math.atan2(twy, twx)-np.pi)
    wind_num = angle2num(twa-heading)
    
    return tws, twa  ,wind_num


def strategy_select(goal_num, wind_num, u_num):
    #print(goal_num, wind_num, u_num)
    for j in range(128):
        if int(LRX[j,0]) == wind_num and int(LRX[j,1]) == goal_num and int(LRX[j,2]) == u_num:
            strategy_num = j
    return strategy_num

def sa_goal_func(twa, sa_num, heading):
    wha = twa-heading
    if sa_num == 2:
        if wha > 0:
            sa_goal = wha-5/8*np.pi
        else: 
            sa_goal = wha+5/8*np.pi
    elif sa_num == 3:
        if wha > 0:
            sa_goal = 0.5*wha-np.pi*7/16
        else:
            sa_goal = 0.5*wha+np.pi*7/16
    elif sa_num == 4:
        sa_goal = 0
    
    else:
        if wha > 0:
            sa_goal = wha-np.pi*5/8
        else: 
            sa_goal = wha+np.pi*5/8
    return sa_goal

def sa_tacking(twa,heading,awa):
    wha = twa-heading
    if np.abs(wha) > np.pi/4:
        if wha > 0:
            sa_goal = awa-np.pi*5/8
        else: 
            sa_goal = awa+np.pi*5/8
    else:
        if awa > 0:
            sa_goal = awa-np.pi/2
        else: 
            sa_goal = awa+np.pi/2
    return sa_goal


def sa_func(sa_goal, sa_old, u_judgement):
    sa_step = np.pi/60 # 每步转帆角度
    if np.abs(sa_goal) < np.abs(sa_old): # 当前帆角比目标帆角靠外，直接到达目标帆角
        sa_output = sa_goal
    elif np.abs(sa_goal-sa_old) > np.pi/4: # 当前帆角与目标帆角相差较大，直接到达目标帆角
        sa_output = sa_goal
    elif u_judgement == 0: # 当前航速比目标航速小，帆角往内一步
        if sa_goal > 0:
            sa_output = sa_old-sa_step
        else:
            sa_output = sa_old+sa_step
    elif np.abs(sa_goal-sa_old) < sa_step:
        sa_output = sa_goal
    else: # 否则帆角向外增加一步
        if sa_goal > 0:
            sa_output = sa_old+sa_step
        else:
            sa_output = sa_old-sa_step
    return sa_output # 返回控制帆角值

# ra_num:1=保持向目标艏向, 2=保持当前艏向, 3=左转45°, 4=右转45°
def heading_goal_func(goal_angle, ra_num, heading,goal_fix):
    if ra_num == 1:
        heading_goal = goal_angle+heading
    elif ra_num == 2:
        heading_goal = heading
    elif ra_num == 3:
        heading_goal = (goal_fix-np.pi/3)
    else:
        heading_goal = (heading+np.pi/6)

    return heading_goal
def heading_tacking_l(goal_fix):
    heading_goal = angle_limit(goal_fix-np.pi/2)
    return heading_goal

def heading_tacking_r(goal_fix):
    heading_goal = angle_limit(goal_fix+np.pi/2)
    return heading_goal


def ra_func(heading_error, u_judgement):
    ra_limit=0.25
    kp = para_cfg[2]
    kd = para_cfg[4]
    ki = para_cfg[3]
    if u_judgement == 0:
        ra_output = 0
    else:
        ra_output = kp*heading_error[-1]+kd*(heading_error[-1]-heading_error[-2])+ki*(sum(heading_error))
        if ra_output>ra_limit:
            ra_output=ra_limit
        elif ra_output<-ra_limit:
            ra_output=-ra_limit
    return ra_output


# 角度限制（限制在(-pi, pi]范围内）
def angle_limit(angle):
    if angle > np.pi:
        angle -= 2*np.pi
    if angle <= -np.pi:
        angle += 2*np.pi
    return angle

# 根据传感器输入规划帆船的帆角sa和舵角ra

def module(sensor_submsg,goal,goal_fix):
    global LABEL, label_tacking_time
    u, v, r, dheel, x, y, heading, heel,aws,awa=sensor_submsg[0], sensor_submsg[1], sensor_submsg[2], sensor_submsg[3], sensor_submsg[4], sensor_submsg[5], sensor_submsg[6], sensor_submsg[7], sensor_submsg[8], sensor_submsg[9]
    tws, twa, wind_num = aw2tw(aws, awa, u, v, heading)
    u_num = u2num(u, tws)
    goal_angle, goal_num = goal_angle_func(x,y,heading, goal[0],goal[1])
    strategy_num = strategy_select(goal_num, wind_num, u_num)
    temp.append(strategy_num)
    u_numlist.append(u_num)
    awa_list.append(awa)
    strategy[0]=strategy_num
    # print('heading',heading,'twa',twa,'awa',awa)
    if len(u_numlist)>10:
        u_sample = Counter(u_numlist).most_common(1)
        u_judgement=u_sample[0][0]#确定速度是否满足
    else:
        u_judgement=1
    if len(temp)>10:
        maxNum_sample = Counter(temp).most_common(1)
    else:
        maxNum_sample = (Counter(temp).most_common(1))
    if maxNum_sample[0][1]>5:#选择策略
        strategy.append(maxNum_sample[0][0])
    else:
        strategy.append(strategy[-1])

    if strategy[-1]!=strategy[-2]:
        goal_fix=heading
        heading_error.clear()
        heading_error.append(0)
    
    label_tacking=LRX[strategy[-1],5]

    if LABEL==1:
        sa_goal=sa_tacking(twa,heading,awa) 
        if goal_angle<0:
            heading_goal=heading_tacking_l(heading)
        else:
            heading_goal=heading_tacking_r(heading)
        # if heading<heading_limt[-1]-np.pi/4 or heading>heading_limt[-1]+np.pi/4:
        #   LABEL=0
        #   label_tacking_time=-200
        #   heading_limt.clear
        if twa-heading_limt[-1]>0:
            if heading>heading_limt[-1]+np.pi/4 or heading<heading_limt[-1]-np.pi/30:
                LABEL=0
                label_tacking_time=-200
                heading_limt.clear
        else:
            if heading<heading_limt[-1]-np.pi/4 or heading>heading_limt[-1]+np.pi/30:
                LABEL=0
                label_tacking_time=-200
                heading_limt.clear
    else:
        if label_tacking == 1 and label_tacking_time>0:
            LABEL=1
            heading_limt.append(heading)
        label_tacking_time+=1
        sa_num=LRX[strategy[-1],3]
        ra_num=LRX[strategy[-1],4]
        sa_goal=sa_goal_func(twa,sa_num, heading)
        heading_goal=heading_goal_func(goal_angle,ra_num,heading,goal_fix)



    label.append(label_tacking)
    sa_output=sa_func(sa_goal,sa_out[-1],u_judgement)
    sa_out.append(sa_output)
    heading_error.append(heading_goal-heading)
    ra_output=ra_func(heading_error ,u_judgement)

    print(x,y)
    return sa_output,ra_output

def getOutMachPut(msg): #sailboat_message::Mach_msg
    mach_pub = Mach_msg()
    mach_pub.header.stamp = rospy.Time.now()
    mach_pub.header.frame_id = 'AHRS'
    #mach_pub.timestamp = rospy.Time.now()
    mach_pub.motor = 0
    mach_pub.rudder = msg[0]
    mach_pub.sail   = msg[1]
    mach_pub.PCCtrl = msg[2]
    return mach_pub


def getOutput(msg): #spare_function::spare_function_out
    out_pub = spare_function_out()
    out_pub.rudder = msg[0]
    out_pub.sail = msg[1]
    return out_pub


def getOutParaPut(msg):#spare_function::spare_function_para
    para_pubmsg = spare_function_para()
    para_pubmsg.oyaw   = msg[1]
    para_pubmsg.rudderP= msg[2]
    para_pubmsg.rudderI= msg[3]
    para_pubmsg.rudderD= msg[4]
    para_pubmsg.sailP  = msg[5]
    para_pubmsg.sailI  = msg[6]
    para_pubmsg.sailD  = msg[7]
    return para_pubmsg

def sensorCallback(msg): #sailboat_message::Sensor_msg
    global sensor_submsg 
    sensor_submsg[0] = msg.ux
    sensor_submsg[1] = msg.vy
    sensor_submsg[2] = msg.gz
    sensor_submsg[3] = msg.gx
    sensor_submsg[4] = msg.Posx
    sensor_submsg[5] = msg.Posy
    sensor_submsg[6] = msg.Yaw
    sensor_submsg[7] = msg.Roll
    sensor_submsg[8] = msg.AWS
    sensor_submsg[9] = msg.AWA


def getConfigCallback(config, level): # spare_function::spare_function_Config
    global para_cfg
    if (config.PC_Ctrl == True):
        para_cfg[0] = 1
    else:
        para_cfg[0] = 0
    para_cfg[1] = config.oyaw
    para_cfg[2] = config.rudderP
    para_cfg[3] = config.rudderI
    para_cfg[4] = config.rudderD
    para_cfg[5] = config.sailP
    para_cfg[6] = config.sailI
    para_cfg[7] = config.sailD
    return config

if __name__ == "__main__":
    rospy.init_node("example", anonymous = True)

    mach_pub = rospy.Publisher('mach', Mach_msg, queue_size=5)
    spare_function_pub = rospy.Publisher('spare_function_out', spare_function_out, queue_size=5)
    spare_function_para_pub = rospy.Publisher('spare_function_para', spare_function_para, queue_size=5)
    
    rospy.Subscriber("sensor", Sensor_msg, sensorCallback)
    config_srv = Server(spare_function_Config, getConfigCallback)

    rate = rospy.Rate(10)
    goal_list = [(20,0),(-55,70),(10,-10)]
    k = 0
    goal_fix = 0
    try:
        while not rospy.is_shutdown():
	    print(sensor_submsg)
            sa, ra = module(sensor_submsg, goal_list[k], goal_fix)
            distance=math.sqrt((goal_list[k][0]-sensor_submsg[4])*(goal_list[k][0]-sensor_submsg[4])+(goal_list[k][1]-sensor_submsg[5])*(goal_list[k][1]-sensor_submsg[5]))
            if distance<3:
                k=k+1

            if k>len(goal_list):
                k=0




            mach_np = [ra, sa, 1]
            out_np = [ra, sa]

            # input : sensor_submsg 
            # cfg: para_cfg
            # output : mach_np out_np para_np

            mach_pubmsg = getOutMachPut(mach_np)
            out_pubmsg = getOutput(out_np)
            para_pubmsg = getOutParaPut(para_cfg)

            mach_pub.publish(mach_pubmsg)
            spare_function_pub.publish(out_pubmsg)
            spare_function_para_pub.publish(para_pubmsg)

            rate.sleep()
    except rospy.ROSInterruptException:
        pass
    #finally:
        #close()
    rospy.spin()
