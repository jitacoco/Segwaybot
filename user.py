#
# import pyb
# from pyb import LED
# from oled_938 import OLED_938
# from mpu6050 import MPU6050
#
# b_LED = LED(4)
#
# oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64,
#                    external_vcc=False, i2c_devid=61)
# oled.poweron()
# oled.init_display()
#
# # IMU connected to X9 and X10
# imu = MPU6050(1, False)    	# Use I2C port 1 on Pyboard
#
# def read_imu(dt):
# 	global g_pitch
# 	alpha = 0.7    # larger = longer time constant
# 	pitch = int(imu.pitch())
# 	roll = int(imu.roll())
# 	g_pitch = alpha*(g_pitch + imu.get_gy()*dt*0.001) + (1-alpha)*pitch
# 	# show graphics
# 	oled.clear()
# 	oled.line(96, 26, pitch, 24, 1)
# 	oled.line(32, 26, g_pitch, 24, 1)
# 	oled.draw_text(0,0," Raw | PITCH |")
# 	oled.draw_text(83,0, "filtered")
# 	oled.display()
#
# g_pitch = 0
# tic = pyb.millis()
# while True:
# 	b_LED.toggle()
# 	toc = pyb.millis()
# 	read_imu(toc-tic)
# 	tic = pyb.millis()

execfile('Milestone_2_Ori.py')
