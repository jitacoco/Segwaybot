# ----------------------------------------------------
#  Using BLUEFRUIT board to control speed of motor

import pyb
from pyb import Pin, Timer, ADC, UART
from oled_938 import OLED_938
from motor import MOTOR

print('BlueFruit Test')


Segway = MOTOR()

#initialise UART communication
uart = UART(6)
uart.init(9600, bits=8, parity = None, stop = 2)

# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
try:
	i2c_2 = pyb.I2C(2, pyb.I2C.MASTER)
	dev_list =i2c_2.scan()
	oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64,
                   external_vcc=False, i2c_devid=dev_list[0])
	oled.poweron()
	oled.init_display()
	oled_ok = True
except:
	print("\n*** OLED Display missing ***\n")



# Use keypad U and D keys to control speed
while True:				# loop forever until CTRL-C
	while (uart.any()!=10):    #wait we get 10 chars
		n = uart.any()
	command = uart.read(10)
     

	

	if command[2]==ord('1'):
		oled.draw_text(0, 30, ' 1-key pressed')
		print('1-key pressed')
		
		Segway.A_stop()
		Segway.B_stop()
		

	elif command[2]==ord('2'):
		oled.draw_text(0, 30, ' 2-key pressed')
		print('2-key pressed')

	elif command[2]==ord('3'):
		oled.draw_text(0, 30, ' 3-key pressed')
		print('3-acceleration')	
		Segway.up_Aspeed(10)
		Segway.up_Bspeed(10)
		
	

	elif command[2]==ord('4'):
		oled.draw_text(0, 30, ' 4-key pressed')
		print('4-decceleration')
		Segway.dn_Aspeed(10)
		Segway.dn_Bspeed(10)
		
	

	elif command[2]==ord('5'):
		oled.draw_text(0, 30, 'UP key pressed')
		print('UP key pressed')
		Segway.A_forward(20)
		Segway.B_forward(20)


	elif command[2]==ord('6'):
		oled.draw_text(0, 30, 'DN key pressed')
		print('DN key pressed')
		Segway.A_back(Segway.Aspeed)
		Segway.B_back(Segway.Bspeed)
		
	elif command[2]==ord('7'):
		oled.draw_text(0, 30, 'LF key pressed')
		print('LF key pressed')
		Segway.up_Aspeed(20)
		Segway.up_Bspeed(0)

	elif command[2]==ord('8'):
		oled.draw_text(0, 30, 'RT key pressed')
		print('RT key pressed')
		Segway.up_Aspeed(0)
		Segway.up_Bspeed(20)
	oled.display()
