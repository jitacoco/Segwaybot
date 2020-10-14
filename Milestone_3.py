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
from pyb import Pin, Timer, ADC, LED
from array import array				    # need this for memory allocation to buffers
from oled_938 import OLED_938			# Use OLED display driver
from mpu6050 import MPU6050             # IMU
from mic import MICROPHONE			    # Peter's microphone functions


micropython.alloc_emergency_exception_buf(100)

# -------- PERIPHERAL SETUP -------- #

# OLED Display
# I2C connected to Y9, Y10 (I2C bus 2) and Y11 is reset low active
oled = OLED_938(pinout={'sda': 'Y10', 'scl': 'Y9', 'res': 'Y8'}, height=64,
                   external_vcc=False, i2c_devid=61)
oled.poweron()
oled.init_display()
oled.draw_text(0,20, 'Milestone 3: Loading...')
oled.display()

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
    w_t = (k_p*error) + (k_d*pit_dot) + (k_i*pit_error)  # compute necessary speed using PID equation
    pit_error += error              # cumulative error adjust
   
    if w_t >= 100:                    # limit w to +-100% of PWM
        w_t = 100
    elif w_t <= -100:
        w_t = -100
    return w_t


# ----- MAINLOOP CONSTANTS ----- #
M = 50					        # number of instantaneous energy epochs to sum
BEAT_THRESHOLD = 4.5			# threshold for c to indicate a beat
SILENCE_THRESHOLD = 2.5		    # threshold for c to indicate silence
alpha = 0.95                    # filter alpha value
motor_offset = 5                # remove motor deadzone
pitch_offset = 1.4              # counteract centre of mass: -ve = lean forward


# ----- K Scalars ----- #
k_p = 10  # Proportional
k_i = 0.5  # Integral
k_d = 0.5 # Derivative

# ----- MAINLOOP VARIABLES ----- #
count = 0                        # position in movelist (counter)
dance_list = []                      # empty list of moves to add to
next_step = 'H'                     # step to execute (initially stopped)
e_ptr = 0                           # pointer to energy buffer
e_buf = array('L', 0 for i in range(M))     # reserve storage for energy buffer
sum_energy = 0                              # total energy in last 50 epochs
pyb.delay(100)
pitch = 0                           # initial pitch angle
target = pitch_offset               # initial target angle
pit_error = 0                       # start with 0 cumulative error
move_speed = 20


# ------- IMPORT MOVES -------- #
with open('bgm_2.txt') as f:
    for line in f:
        dance_list.append(line.strip())
print(dance_list)
move_num = len(dance_list)

#   'move': [ new target angle, A speed multiplier, B speed multiplier ]
steps = {
    'F': [pitch_offset - 0.5, 1, 1],        # forward
    'B': [pitch_offset + 0.5, 1, 1],        # back
    'CC': [pitch_offset + 0.5, 1, 0.1],
    'C': [pitch_offset + 0.5, 0.1, 1],
    'L': [pitch_offset - 0.5, 0.4, 1],      # left
    'R': [pitch_offset - 0.5, 1, 0.4],      # right
    'LF': [pitch_offset - 0.2, 0.6, 1],     # left slow
    'FR': [pitch_offset - 0.2, 1, 0.9],
    'LR': [pitch_offset + 0.2, 1, 0.6],     # right slow
    'BL': [pitch_offset + 0.2, 1, 0.8],
    'H': [pitch_offset, 1, 1]               # stop
}


# ------- IDLE STATE -------- #
print('Ready to begin Milestone 3')
print('Waiting for button press')   #debug
oled.clear()
oled.draw_text(5, 20, 'MILESTONE 3: Ready')
oled.draw_text(5, 40, 'Press USR button')
oled.display()
trigger = pyb.Switch()				       # Trigger on USR button
while not trigger():				       # Wait for button press
   time.sleep(0.001)
while trigger(): pass
print('Button pressed - running')
oled.clear()
oled.draw_text(5, 20, 'MILESTONE 3')
oled.draw_text(20, 40, 'Running...')
oled.display()

tic2 = pyb.millis()                        # start tic2 (used for beat)

# ----- MAIN PROGRAM LOOP ----- #
try:
    tic1 = pyb.micros()                    # start tic1 (used for balance)
    while True:
        # ----- PID SECTION
        dt = pyb.micros() - tic1
        if dt > 5000:  # wait for sampling time
            pitch, pitch_dot = pitch_angle(pitch, dt * 0.000001, alpha)
            tic1 = pyb.micros()
            pid = pid_controller(pitch, pitch_dot, target)

            # -- SPEED -- #
            speed = ((abs(pid) + motor_offset)/1.4)
            print(pid)
            
            motorA.pulse_width_percent(speed * steps[next_step][1])    # instant speed adjusted for turning
            motorB.pulse_width_percent(speed * steps[next_step][2])
           

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

        # ----- BEAT DETECTION
        if mic.buffer_full:     # semaphore signal from ISR - set if buffer is full

            # Calculate instantaneous energy
            E = mic.inst_energy()

            # compute moving sum of last 50 energy epochs
            sum_energy = sum_energy - e_buf[e_ptr] + E
            e_buf[e_ptr] = E            # over-write earlest energy with most recent
            e_ptr = (e_ptr + 1) % M     # increment e_ptr with wraparound - 0 to M-1

            # Compute ratio of instantaneous energy/average energy
            c = E * M / sum_energy
            # look for a beat
            if (pyb.millis() - tic2 > 545):             # if more than 545ms since last beat -
                if (c > BEAT_THRESHOLD):                # look for a beat
                    # print(c,'c is')
                    flash()                             # beat found, flash blue LED
                    next_step = dance_list[count]     # move to next dance move in list
                    
                    # move_functions[next_step][1](speed)
                    target = steps[next_step][0]
                    print(next_step)                              # debug
                    position = (position + 1) % move_num          # ready for next move
                    tic2 = pyb.millis()                           # reset tic2
            mic.set_buffer_empty()                                # reset the buffer_full flag

finally:
    A1.high(), A2.high()
    B1.high(), B2.high()
    print('High')
