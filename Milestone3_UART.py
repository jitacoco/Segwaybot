'''    Start of comment section
-------------------------------------------------------
Name: Milestone_3_Ori
Based on the work of the honorable Peter YK Cheung
Date:   4 March 2020
Revision:  1
-------------------------------------------------------
'''
import pyb
import time
import micropython
from pyb import Pin, Timer, ADC, LED, UART
from array import array				    # need this for memory allocation to buffers
from oled_938 import OLED_938			# Use OLED display driver
from mpu6050 import MPU6050             # IMU
from mic import MICROPHONE			    # Peter's microphone functions

micropython.alloc_emergency_exception_buf(100)

# -------- PERIPHERAL SETUP -------- #

# -- Initiate OLED
# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64,
                   external_vcc=False, i2c_devid=61)
oled.poweron()
oled.init_display()
oled.draw_text(0,20, 'Milestone 3: Loading...')
oled.display()

# -- Initiate UART
uart = UART(6)
uart.init(9600, bits=8, parity=None, stop=2)


# -- Microphone, 8000 Hz sample rate
N = 160
mic = MICROPHONE(Timer(7,freq=8000),ADC('Y11'),N)

# -- Blue LED
b_LED = LED(4)

# -- IMU
imu = MPU6050(1, False)
pot = ADC(Pin('X11'))
usr = pyb.Switch()

# -- WHEELS
A1 = Pin('X3', Pin.OUT_PP)		    # Control direction of motor A
A2 = Pin('X4', Pin.OUT_PP)
PWMA = Pin('X1')				    # Control speed of motor A
B2 = Pin('X7', Pin.OUT_PP)		    # Control direction of motor B
B1 = Pin('X8', Pin.OUT_PP)
PWMB = Pin('X2')				    # Control speed of motor B

# -- TIMERS
tim = Timer(2, freq = 1000)
motorA = tim.channel (1, Timer.PWM, pin = PWMA)
motorB = tim.channel (2, Timer.PWM, pin = PWMB)

# -- initialise UART communication
uart = UART(6)
uart.init(9600, bits=8, parity = None, stop = 2)

# ----- FLASH SCRIPT ----- #
def flash():
    b_LED.on()
    pyb.delay(20)
    b_LED.off()

# ----- PID FUNCTIONS ----- #
def pitch_angle(pitch, dt, alpha):  # estimate pitch angle
    theta = imu.pitch()             # take angle from imu
    pitch_dot = imu.get_gy()        # take pitch change from gyro
    pitch = alpha*(pitch + pitch_dot*dt) + (1-alpha)*theta  # complimentary filter
    return (pitch, pitch_dot)       # return pitch & rate of change of pitch


def pid_controller(pit, pit_dot, target):
    global pit_error                # allow function to modify overall pitch error
    error = pit - target            # find instant error
    w = (kp*error) + (kd*pit_dot) + (ki*pit_error)  # compute necessary speed using PID equation
    pit_error += error              # cumulative error adjust
    if w >= 100:                    # limit w to +-100% of PWM
        w = 100
    elif w <= -100:
        w = -100
    return w


# ----- MAINLOOP CONSTANTS ----- #
alpha = 0.95
motor_offset = 5                # remove motor deadzone
pitch_offset = 1.8             # counteract centre of mass: -ve = lean forward

# ----- K Scalars ----- #
kp = 15   # Proportional
ki = 0.1   # Integral
kd = 0.5   # Derivative

# ----- MAINLOOP VARIABLES ----- #
pyb.delay(100)
pitch = 0                           # initial pitch angle
target = pitch_offset               # initial target angle
pit_error = 0                       # start with 0 cumulative error
move_speed = 20
A_scale = 1
B_scale = 1

#   'move': [ new target angle, A speed multiplier, B speed multiplier ]
move_steps = {
    'F': [pitch_offset - 0.5, 1, 1],        # forward
    'B': [pitch_offset + 1.1, 1, 1],        # back
    'CC': [pitch_offset + 0.5, 1, 0.1],
    'C': [pitch_offset + 0.5, 0.1, 1],
    'L': [pitch_offset - 0.5, 0.4, 1],      # left
    'R': [pitch_offset - 0.5, 1, 0.4],      # right
    'FL': [pitch_offset - 0.2, 0.8, 1],     # left slow
    'FR': [pitch_offset - 0.2, 1, 0.8],
    'BR': [pitch_offset + 0.2, 1, 0.8],     # right slow
    'BL': [pitch_offset + 0.2, 1, 0.8],
    'H': [pitch_offset, 1, 1]               # stop
}


# ------- IDLE STATE -------- #
print('Ready to begin Milestone 4')
print('Waiting for button press')   #debug
oled.clear()
oled.draw_text(5, 20, 'Balanced UART: Ready')
oled.draw_text(5, 40, 'Press USR button')
oled.display()
trigger = pyb.Switch()				# Trigger on USR button
while not trigger():				  # Wait for button press
   time.sleep(0.001)
while trigger(): pass
print('Button pressed - running')
oled.clear()
oled.draw_text(5, 20, 'Balanced UART: Ready')
oled.draw_text(20, 40, 'Running...')
oled.display()

# ----- MAIN PROGRAM LOOP ----- #
try:
    tic = pyb.micros()             # start tic1 (used for balance)
    while True:
        # ----- PID SECTION
        dt = pyb.micros() - tic
        if dt > 5000:  # wait for sampling time
            # -- KEYPAD CONTROL -- #
            if uart.any() == 5:
                command = uart.read(5)
                if command[2] == ord('5'): # press forward key to nudge forward
                    pitch_offset += 5
                    A_scale = 1
                    B_scale = 1
					#oled.draw_text(0, 30, 'forward pressed')
                    print('forward')
                if command[2] == ord('6'): # backward
                    pitch_offset -= 1.2
                    A_scale = 1
                    B_scale = 1
					#oled.draw_text(0, 30, 'back')
                    print('back')
                if command[2] == ord('7'): # left
                    pitch_offset += 0.5
                    A_scale = 0.6
                    B_scale = 1
					#oled.draw_text(0, 30, 'left')
                    print('left')
                if command[2] == ord('8'): # right
                    pitch_offset += 0.5
                    A_scale = 1
                    B_scale = 0.6
					#oled.draw_text(0, 30, 'right')
                    print('right')
					#oled.display()
            
            pitch, pitch_dot = pitch_angle(pitch, dt * 0.000001, alpha)
            pid = pid_controller(pitch, pitch_dot, target)
            
            # -- SPEED -- #
            
            speed = ((abs(pid) + motor_offset)/1.4)
            # print(pid)
            
            motorA.pulse_width_percent(speed * A_scale)    # instant speed adjusted for turning
            motorB.pulse_width_percent(speed * B_scale)
            
            if pid > 0:     # go forwards
                A1.high()
                A2.low()
                B1.high()
                B2.low()
            elif pid < 0:   # go backwards
                A1.low()
                A2.high()
                B1.low()
                B2.high()
            else:           # stay still
                A1.high(), A2.high()
                B1.high(), B2.high()
            
            tic = pyb.micros()


finally:
    A1.high(), A2.high()
    B1.high(), B2.high()
    print('Stopped')