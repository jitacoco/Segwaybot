'''
-------------------------------------------------------
User's Program - put your code here
'''

import pyb
import gc
from pyb import Pin, Timer, LED
from oled_938 import OLED_938
from struct import unpack

# Constants
BLANK_LINE          = "                   "

# Initialise OLED display
try:
	oled_port = pyb.I2C('Y',pyb.I2C.MASTER)
	if (not oled_port.scan()):
		oled = None
	else:
		oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'},
			height=64, external_vcc=False, i2c_devid=61)
		oled.poweron()
		oled.init_display()
 		oled.draw_text(0, 0, "-- User's Program --")
		oled.draw_text(0,40, "--  Edit user.py  --")
		oled.display()
		b_LED = LED(4)
		b_LED.on()
except ValueError:
	pass
