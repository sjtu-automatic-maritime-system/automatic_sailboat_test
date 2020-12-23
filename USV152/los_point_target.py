#!/usr/bin/env python
import math
from math import pi, sin, cos, sqrt
import time

from msgdev import MsgDevice
from PDcontroller import Controller2Trimaran 

POS_X = 0
POS_Y = 1
YAW = 2
YAW_SPEED = 3
SPD = 4
SPD_DIR = 5

class Interface(object):

    def __init__(self, sub_addr, ahrs_port, gnss_port, ekf_port, motor_port):
        self.dev = MsgDevice()
        self.dev.open()
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
        
        self.dev.sub_connect(sub_addr+':'+ekf_port)        
        self.dev.sub_add_url('USV150.state', default_values=[0,0,0,0,0,0])
        
        self.dev.pub_bind('tcp://0.0.0.0:'+motor_port)

    def receive1(self, *args):
        data = []
        for i in args:
            data.append(self.dev.sub_get1(i))
        return data
    
    def receive(self, *args):
        data = []
        for i in args:
            data.append(self.dev.sub_get(i))
        return data

    def Motor_send(self, left_motor, right_motor):
        print ('left_motor, right_motor', left_motor, right_motor)
        self.dev.pub_set1('pro.left.speed', left_motor)
        self.dev.pub_set1('pro.right.speed', right_motor)
    
    def pointSend(self, pose):
        self.dev.pub_set('target.point', pose)

def ship_initialize(USE_TLG001, USE_TLG002):
    if USE_TLG001:
        sub_addr1 = 'tcp://192.168.1.150'  # 'tcp://127.0.0.1'
        ahrs_port1 = '55005'
        gnss_port1 = '55004'
        ekf_port1 = '55007'
        motor_port1 = '55002'
        interface001 = Interface(sub_addr1, ahrs_port1, gnss_port1, efk_port1, motor_port1)
    else:
        interface001 = None

    if USE_TLG002:
        sub_addr2 = 'tcp://192.168.1.152'  
        # sub_addr2 = 'tcp://127.0.0.2'
        ahrs_port2 = '55205'
        gnss_port2 = '55204'
        ekf_port2 = '55207'
        motor_port2 = '55202'
        interface002 = Interface(sub_addr2, ahrs_port2, gnss_port2, ekf_port2, motor_port2)
    else:
        interface002 = None

    return interface001, interface002

def calPointFromPointLine(point1, point2, self_point):
    angle = math.atan2(point2[1]-point1[1], point2[0]-point1[0])
    if math.fabs(angle)<0.1:
        x = self_point[0]
        y = point1[1]
    else:
        x = (self_point[1]-point1[1]+math.tan(angle)*point1[0]+self_point[0]/math.tan(angle))/(math.tan(angle)+1/math.tan(angle))
        y = point1[1]+math.tan(angle)*(x-point1[0])
    return [x,y]


def calPoint(targets, current, slef_pos):
    dis = 5
    
    if current == len(targets)-1:
        next_id = 0
    else:
        next_id = current+1 
    point_close = calPointFromPointLine(targets[current], targets[next_id], slef_pos)

    run_yaw = math.atan2(targets[current][1]-point_close[1], targets[current][0]-point_close[0])

    dis_x = math.cos(run_yaw)*dis
    dis_y = math.sin(run_yaw)*dis
    

    point_farther = [point_close[0]+dis_x, point_close[1]+dis_y]

    return point_farther, point_close, run_yaw

def calPosDis(pos_close, self_pos):
    disL = math.sqrt(math.pow(pos_close[0]-self_pos[0],2)+math.pow(pos_close[1]-self_pos[1],2))
    target_yaw = math.atan2(pos_close[1]-self_pos[1], pos_close[0]-self_pos[0])

    target_u = min(disL*0.5, 1)
    return disL, target_yaw, target_u

def calLosDis(target, point, pos_farther, self_pos, run_yaw):

    dis_L_y = ((self_pos[0]-point[0])*(point[1]-pos_farther[1])-(self_pos[1]-point[1])*(point[0]-pos_farther[0])) / math.sqrt(math.pow(point[1]-pos_farther[1], 2)+math.pow(point[0]-pos_farther[0], 2))
    dis_L_u = - ((target[0]-point[0])*(point[1]-self_pos[1])-(target[1]-point[1])*(point[0]-self_pos[0])) / math.sqrt(math.pow(point[1]-self_pos[1], 2)+math.pow(point[0]-self_pos[0], 2))
    dis_L_u = math.fabs(dis_L_u)
    target_yaw = run_yaw - math.atan2(dis_L_y, max(5, dis_L_u))
    target_u = min(dis_L_u*0.1, 0.5)

    return dis_L_y, dis_L_u, target_yaw, target_u


if __name__ == "__main__":
    rate = 10
    _, interface002 = ship_initialize(False, True)

    target_points = [[0, 40], [45, -15]]
    current = 0
    state = 1
    controller = Controller2Trimaran()

    try:
        while True:
            start = time.time()
            sensor_state = interface002.receive('USV150.state')
            sensor_state = sensor_state[0]
            
            #[u,v,r,x,y,yaw]
            #continue


            target_point = target_points[current]
            sensor_submsg = [sensor_state[3], sensor_state[4], sensor_state[5], sensor_state[0], sensor_state[1]]
            print ('x: ', sensor_submsg[0], 'y: ', sensor_submsg[1], 'yaw:', sensor_submsg[2])
            print ('state', state, 'current', current)
            if state == 0:

                disL, target_yaw, target_u = calPosDis(target_point, sensor_submsg)
                print ('target_yaw: ', target_yaw, 'target_u: ', target_u)
                if (math.fabs(disL) < 1):
                    state += 1
                    current += 1
                else:
                    self_u = math.sqrt(math.pow(sensor_submsg[3], 2)+math.pow(sensor_submsg[4], 2))
                    left_motor,right_motor = controller.outputSignal(target_yaw, sensor_submsg[2], target_u, self_u)
                    interface002.Motor_send(-left_motor,right_motor)
                    #print ('left_motor,right_motor ', left_motor,right_motor)
            if state == 1:
                pos_farther, pos, run_yaw = calPoint(target_points, current, sensor_submsg)
                dis_L_y, dis_L_u, target_yaw, target_u = calLosDis(target_point, pos, pos_farther, sensor_submsg, run_yaw)

                print ('pos_farther, pos, run_yaw: ',pos_farther, pos, run_yaw)
                print ('dis_L_y, dis_L_u, target_yaw, target_u', dis_L_y, dis_L_u, target_yaw, target_u)
                if (math.fabs(dis_L_u) < 1):
                    current += 1
                else:
                    self_u = math.sqrt(math.pow(sensor_submsg[3], 2)+math.pow(sensor_submsg[4], 2))
                    left_motor,right_motor = controller.outputSignal(target_yaw, sensor_submsg[2], target_u, self_u)
                    interface002.Motor_send(-left_motor,right_motor)
            if current == len(target_points):
                current = 0
            print(current)
            point = target_points[current]
            interface002.pointSend(point)
            end = time.time()
            if (end-start)<1./rate:
                sleep = 1./rate - (end-start)
                #print ('sleep: ', sleep)
                time.sleep(sleep)

    finally:
        interface002.Motor_send(0, 0)
        time.sleep(0.5)
        interface002.dev.close()
