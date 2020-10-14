'''
Name: Milestone 3 - Balance-Dance
Creator:  Group 20
Date:   March 2018
Revision:  ?
-------------------------------------------------------
Drive motors to perform dance to beat while balanced
-------------------------------------------------------
'''

import pyb
from pyb import Pin, Timer, ADC, DAC, LED, ExtInt
import time
from array import array			# need this for memory allocation to buffers
from oled_938 import OLED_938	# Use OLED display driver
from mpu6050 import MPU6050
from motor import MOTOR
import mic
import micropython
micropython.alloc_emergency_exception_buf(100)
var = False
var2 = False
pyb.disable_irq()			# disable interrupt while configuring timer


Segway = MOTOR()

# Create timer interrupt - one every 1/8000 sec or 125 usec



# Use OLED to say what segway is doing
oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64, external_vcc=False, i2c_devid=61)
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Group 20')
oled.draw_text(0, 10, 'Milestone3: ALL')
oled.draw_text(0, 20, 'Press USR button')
oled.display()

print('Performing Milestone 3')
print('Waiting for button press')

trigger = pyb.Switch()		# Create trigger switch object
while not trigger():		# Wait for trigger press
	time.sleep(0.001)
while trigger():
	pass			# Wait for release
print('Button pressed - Running')
oled.draw_text(0, 30, 'Button pressed - Running')
oled.display()

# --- Microphone set up --- #
# Define ports for microphone, LEDs and trigger out (X5)
mic = ADC(Pin('Y11'))
MIC_OFFSET = 1527		# ADC reading of microphone for silence
dac = pyb.DAC(1, bits=12)  # Output voltage on X5 (BNC) for debugging
b_LED = LED(4)		# flash for beats on blue LED

N = 160				# size of sample buffer s_buf[]
s_buf = array('H', 0 for i in range(N))  # reserve buffer memory
ptr = 0				# sample buffer index pointer
buffer_full = False	# semaphore - ISR communicate with main program

def energy(buf):	# Compute energy of signal in buffer
	sum = 0
	for i in range(len(buf)):
		s = buf[i] - MIC_OFFSET	# adjust sample to remove dc offset
		sum = sum + s*s			# accumulate sum of energy
	return sum

# ---- The following section handles interrupts for sampling data -----

# Interrupt service routine to fill sample buffer s_buf
def isr_sampling(dummy): 	# timer interrupt at 8kHz
	global ptr				# need to make ptr visible inside ISR
	global buffer_full		# need to make buffer_full inside ISR
	global var
	var = True
	s_buf[ptr] = mic.read()	# take a sample every timer interrupt
	global var2
	var2 = True
	ptr += 1				# increment buffer pointer (index)
	if (ptr == N):			# wraparound ptr - goes 0 to N-1
		ptr = 0
		buffer_full = True	# set the flag (semaphore) for buffer full

sample_timer = pyb.Timer(7, freq=4600)	# set timer 7 for 8kHz
sample_timer.deinit()

sample_timer.init(freq=4600)
sample_timer.callback(None)
sample_timer.callback(isr_sampling)		# specify interrupt service routine

	

# -------- End of interrupt section ----------------

# Define constants for main program loop - shown in UPPERCASE
M = 50						# number of instantaneous energy epochs to sum
BEAT_THRESHOLD = 3.6		# threshold for c to indicate a beat (original 2.0)
SILENCE_THRESHOLD = 0.5		# threshold for c to indicate silence

# initialise variables for main program loop
e_ptr = 0					# pointer to energy buffer
e_buf = array('L', 0 for i in range(M))	# reserve storage for energy buffer
sum_energy = 0				# total energy in last 50 epochs
pyb.delay(100)

# --- LED set up --- #
def flash():		# routine to flash blue LED when beat detected
	b_LED.on()
	pyb.delay(30)
	b_LED.off()
	
# Read text file with choreographed dance moves'''
raw = open("bgm.txt","r") #txt file containing letters determining the dance sequence to be acted on
contents = raw.read()
moves = contents.splitlines()
print(moves)
'''
def dance_steps(filename):
	char_list = [ch for ch in open(filename).read() if ch not in ['\r', '\n']]
	return char_list
'''

# DanceMoveList = dance_steps('bgm.txt') #debug
global counter 
counter = 0
def dance(moves, i, v):
	print(i)
	step = moves[i]
	global counter 
	print(step)
	print(v)
	
	if step == 'f':
		print('forward: 1 step')
		balanceDanceA(-v+3)
		balanceDanceB(-v-3)
		#counter += 1

	elif step =='b':
		print('back:1 step')
		balanceDanceA(v)
		balanceDanceB(v-3)
		#counter += 1

	elif step == 's':
		print('Stop')
		balanceDanceA(-v)
		balanceDanceB(-v)
		#counter += 1
    
def balanceDanceA(a):
	a=a+motor_offset
	if a > 100:
		a = 100
	elif a< -100:
	    a = -100
	elif a>0:
		a=a+ motor_offset
	else:
		a=a- motor_offset

	if (a>=0):
		Segway.A_forward(a)
	else:
		Segway.A_back(-a)

def balanceDanceB(b):

	if b >100:
		b = 100
	elif b < -100:
		b = -100
	elif b>0:
		b=b+motor_offset
	else:
		b=b-motor_offset

	if (b>=0):
		Segway.B_forward(b)
	else:
		Segway.B_back(-b)
		

# --- Balance set up --- #
imu = MPU6050(1, False)

# Pitch angle calculation using complementary filter
def pitch_estimation(pitch, dt, alpha):
    theta = imu.pitch()
    pitch_dot = imu.get_gy()
    pitch = alpha*(pitch + pitch_dot*dt) + (1-alpha)*theta
    #print(pitch)
    #print("filtered = " + str(pitch))
    #print("imu = " + str(theta))
    return (pitch, pitch_dot)

alpha = 0.95
pitch = 0
r = - 1.10
#e_int = 0
#e_diff = 0
error = 0
v = 0
K_p = 5.5
K_d = 0.22 #0.22
K_i = 0.35
motor_offset = 5
# --- Main program loop --- #
tic2 = pyb.millis()			# mark time now in msec
pyb.enable_irq()            # enable interrupt again	
pyb.enable_irq()            # enable interrupt again	

try:
	tic1 = pyb.micros()
	while True:				# Main program loop
		dt = pyb.micros() - tic1
		if (dt > 5000):
			if var == True:
				print(ptr)
			if var2 == True:
				print('var2')
				print(mic.read())
			pitch, pitch_dot = pitch_estimation(pitch, dt* 0.000001, alpha)
			tic1 = pyb.micros()

			e_t = pitch - r
			e_dot_t = pitch_dot

			v = (K_p * e_t + K_i * error + K_d * e_dot_t)
			error += e_t
			'''
			if v > 0:    #sending the instruction to  the motor to carry out the self correction
				Segway.A_back(abs(v) + motor_offset)
				Segway.B_back(abs(v) + motor_offset)
			
			elif v < 0:
				Segway.A_forward(abs(v) + motor_offset)
				Segway.B_forward(abs(v) + motor_offset)

	'''
			dance(moves, counter, v)

			#e_diff = e

		if buffer_full:		# semaphore signal from ISR - set if buffer is full
			print('buffer')
	# Calculate instantaneous energy
			E = energy(s_buf)
			# compute moving sum of last 50 energy epochs
			sum_energy = sum_energy - e_buf[e_ptr] + E

			e_buf[e_ptr] = E		# over-write earlest energy with most recent
			e_ptr = (e_ptr + 1) % M	# increment e_ptr with wraparound - 0 to M-1
			print(sum_energy)
			if sum_energy > 50000000:
				# Compute ratio of instantaneous energy/average energy
				c = E*M/sum_energy
				print('energy')
				if (pyb.millis()-tic2 > 540):	# if more than 500ms since last beat
					if (c>BEAT_THRESHOLD):		# look for a beat
						print('beat detected')
						flash()	
						print('*')				# beat found, flash blue LED
						#dance(moves, counter, v)
						counter += 1
						if counter >= len(moves):
							counter = 0
						else:
							tic2 = pyb.millis()		# reset tic
				buffer_full = False				# reset status flag
finally:
	Segway.A_stop()
	Segway.B_stop()