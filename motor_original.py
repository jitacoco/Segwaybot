'''
-------------------------------------------------------
Name: motor.py
Creator:  Peter Y K Cheung, Imperial College London
Date:   17 March 2017
Revision: 1.1
-------------------------------------------------------
Package for driving motors
-------------------------------------------------------
'''
import pyb, micropython
from pyb import Pin, Timer

class MOTOR(object):

	def __init__(self):
		# set up motor with PWM and timer control
		self.A1 = Pin('X3',Pin.OUT_PP)	# A is right motor
		self.A2 = Pin('X4',Pin.OUT_PP)
		self.B1 = Pin('X7',Pin.OUT_PP)	# B is left motor
		self.B2 = Pin('X8',Pin.OUT_PP)
		self.PWMA = Pin('X1')
		self.PWMB = Pin('X2')

		# Configure timer to provide PWM signal
		self.tim = Timer(2, freq = 10000)
		self.motorA = self.tim.channel(1, Timer.PWM, pin = self.PWMA)
		self.motorB = self.tim.channel(2, Timer.PWM, pin = self.PWMB)

		# initialise variables for motor drive strength (PWM values)
		self.Aspeed = 0			# PWM value for motorA
		self.Bspeed = 0			# PWM value for motorB

	def drive(self):	# drove motors at Aspeed and Bspeed
		if self.Aspeed > 0:
			self.A_forward(self.Aspeed)
		else:
			self.A_back(-self.Aspeed)
		if self.Bspeed > 0:
			self.B_forward(self.Bspeed)
		else:
			self.B_back(-self.Bspeed)

	def up_Aspeed(self,value):	# increase motor A speed by value
		self.Aspeed = min(100, self.Aspeed + value)
		self.drive()

	def up_Bspeed(self,value):	# increase motor B speed by value
		self.Bspeed = min(100, self.Bspeed + value)
		self.drive()

	def dn_Aspeed(self,value):	# decrease motor A speed by value
		self.Aspeed = max(-100, self.Aspeed - value)
		self.drive()

	def dn_Bspeed(self,value):	# decrease motor B speed by value
		self.Bspeed = max(-100, self.Bspeed - value)
		self.drive()

	def A_forward(self,value):	# Drive motor A forward by value
		self.Aspeed = min(100, value)
		self.A2.high()
		self.A1.low()
		self.motorA.pulse_width_percent(abs(value))

	def A_back(self,value):		# Drive motor A backward by value
		self.Aspeed = min(100, value)
		self.A2.low()
		self.A1.high()
		self.motorA.pulse_width_percent(abs(value))

	def B_forward(self,value):	# Drive motor B forward by value
		self.Bspeed = min(100, value)
		self.B1.high()
		self.B2.low()
		self.motorB.pulse_width_percent(abs(value))

	def B_back(self,value):		# Drive motor B backward by value
		self.Aspeed = min(100, value)
		self.B1.low()
		self.B2.high()
		self.motorB.pulse_width_percent(abs(value))

	def A_stop(self):			# stop motor A
		self.Aspeed = 0
		self.motorA.pulse_width_percent(0)

	def B_stop(self):			# stop motor B
		self.Bspeed = 0
		self.motorB.pulse_width_percent(0)
