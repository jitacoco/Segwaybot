'''    Marks the start of comment section
-------------------------------------------------------
Name: Show how to file a buffer with sampled data using interrupt
Creator:  Peter YK Cheung
Date:   12 March 2017
Revision:  1.2
-------------------------------------------------------
'''    
import pyb
from pyb import Pin, Timer, ADC, DAC, LED
from array import array
from oled_938 import OLED_938	# Use OLED display driver

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64,
                   external_vcc=False, i2c_devid=61)  
oled.poweron()
oled.init_display()
oled.draw_text(0,0, 'Beat Detection')
oled.display()

# define ports for microphone in and trigger out
mic = ADC(Pin('Y11'))
trigger = Pin('X5',Pin.OUT_PP)

N = 160				# size of sample buffer
s_buf = array('H', 0 for i in range(N))  # reserve buffer memory
global ptr
global buffer_full
ptr = 0				# buffer index
buffer_full = False	# semaphore - ISR communicate with main program

#  Function to plot data on OLED - used only for diagnosis
def	plot_sig(signal,message):
	index = len(signal)
	if index >= 128:
		step = min(index,int(index/128))
	else:
		step = 1
	oled.clear()
	oled.draw_text(0,0,message)
	x = 127
	max_sig = max(max(signal),3000)
	min_sig = min(min(signal),1000)
	range_sig = max_sig - min_sig
	for i in range(0,index,step):
		y = 63 - int((signal[i] - min_sig)*63/range_sig)
		oled.set_pixel(x,y,True)
		x = x - 1
	oled.display()

#  The next two lines are needed by micropython to catch errors
#   ... must include if you use interrupt in your program
import micropython
micropython.alloc_emergency_exception_buf(100)

# Interrupt service routine to fill sample buffer s_buf
def isr_sampling(dummy): 	# timer interrupt at 8kHz
	global ptr				# need to make ptr visible in here
	global buffer_full	# need to make buffer_filled visible in here
	
	s_buf[ptr] = mic.read()		# take a sample every timer interrupt
	ptr += 1
	if (ptr == N):
		ptr = 0
		buffer_full = True
	
# Create timer interrupt - one every 1/8000 sec or 125 usec
sample_timer = pyb.Timer(7, freq=8000)
sample_timer.callback(isr_sampling)

while True:
	if buffer_full:		# semaphore signal from ISR to say buffer full
		plot_sig(s_buf,'Microphone signal')
		if (trigger.value()):	# Measure how long it takes on X5 - top BNC
			trigger.low()
		else:
			trigger.high()
		buffer_full = False
