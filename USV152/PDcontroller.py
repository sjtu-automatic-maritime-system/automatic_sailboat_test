from math import *
import numpy as np 
# import matplotlib.pyplot as plt 

class PD2yaw(object):
	'''
	PD Controller to Yaw Angle
	'''
	def __init__(self, kp=800, kd=100):
		self.kp = kp
		self.kd = kd
		self.old_err = 0

	def pdControl(self, err):
		derr = err - self.old_err
		self.old_err = err
		return self.kp*err + self.kd*derr

	def reset(self):
		self.old_err = 0


class PD2u(object):
	'''
	PD Controller to U Speed
	'''
	def __init__(self, kp=100, kd=10):
		self.kp = kp
		self.kd = kd
		self.old_err = 0

	def pdControl(self, err):
		derr = err - self.old_err
		self.old_err = err
		return self.kp*err + self.kd*derr

	def reset(self):
		self.old_err = 0


class Controller2Trimaran(object):
	'''
	PD Controller to Yaw Angle and U Speed
	'''
	def __init__(self, max_rps=1500, min_rps=0, init_aver=1000):
		self.controller2u = PD2u()
		self.controller2yaw = PD2yaw()
		self.max_rps = max_rps
		self.min_rps = min_rps
		self.aver = init_aver
		self.old_target_yaw = None
		self.old_target_u = None

	def __errYawLimit(self, err_yaw):
		'''
		Restrict Yaw Error within [-pi, pi]
		'''
		if err_yaw > pi:
			err_yaw -= 2*pi
		if err_yaw < -pi:
			err_yaw += 2*pi
		return err_yaw

	def __rpsLimit(self, diff, daver):
		'''
		Restrict Left/Right Motor (rps) within [0, 1500]
		'''
		if  abs(diff) > (self.max_rps-self.min_rps):
			diff = np.sign(diff)*(self.max_rps-self.min_rps)

		if self.aver+daver > self.max_rps:
			self.aver = self.max_rps
		elif self.aver+daver < self.min_rps:
			self.aver = self.min_rps
		else:
			self.aver += daver

		left_motor = self.aver-diff/2
		right_motor = self.aver+diff/2

		if diff >= 0:
			if left_motor < self.min_rps:
				left_motor = self.min_rps
				right_motor = self.min_rps+diff
			if right_motor > self.max_rps:
				right_motor = self.max_rps
				left_motor = self.max_rps-diff
		else:
			if left_motor > self.max_rps:
				left_motor = self.max_rps
				right_motor = self.max_rps+diff
			if right_motor < self.min_rps:
				right_motor = self.min_rps
				left_motor = self.min_rps-diff

		return left_motor, right_motor

	def outputSignal(self, target_yaw, present_yaw, target_u, present_u):

		if target_yaw != self.old_target_yaw:
			self.controller2yaw.reset()
			self.old_target_yaw = target_yaw
		if target_u != self.old_target_u:
			self.controller2u.reset()
			self.old_target_u = target_u

		err_u = target_u - present_u
		err_yaw = -self.__errYawLimit(target_yaw - present_yaw)

		diff = self.controller2yaw.pdControl(err_yaw)
		daver = self.controller2u.pdControl(err_u)
		left_motor, right_motor = self.__rpsLimit(diff, daver)
		return left_motor, right_motor


if __name__ == '__main__':

	flag = 'Trimaran_with_Noise'
	# flag = 'Trimaran_without_Noise'


	controller = Controller2Trimaran()


	#left_motor, right_motor = controller.outputSignal(target_yaw[i], ship.obs_yaw, target_u[i], ship.obs_u)
