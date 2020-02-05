#!/usr/bin/env pybricks-micropython

from pybricks import ev3brick as brick
from pybricks.ev3devices import (Motor, TouchSensor, ColorSensor,
                                 InfraredSensor, UltrasonicSensor, GyroSensor)
from pybricks.parameters import (Port, Stop, Direction, Button, Color,
                                 SoundFile, ImageFile, Align)
from pybricks.tools import print, wait, StopWatch
from pybricks.robotics import DriveBase
import utime, array
from pybricks.ev3devio import Ev3devSensor

class EV3Sensor(Ev3devSensor):
    _ev3dev_driver_name='ev3-analog-01'
    def read(self):
        self._mode('ANALOG')
        return self._value(0)

# this is a hack to set the mode properly
from ev3dev2.port import LegoPort
s2 = LegoPort(address ='ev3-ports:in2')
s2.mode = 'ev3-analog'
utime.sleep(0.5)
s3 = LegoPort(address ='ev3-ports:in3')
s3.mode = 'ev3-analog'
utime.sleep(0.5)

# Write your program here
brick.sound.beep()

# testing
#sensor=EV3Sensor(Port.S1) # same port as above
# higher value means brighter, lower value means darker
#while True:
#    print(sensor.read())
#    wait(2000)

# Plan:
# Start with a bang-bang controller
# Attempt proportional controller
# Attempt proportional-derivative controller

# bang-bang

left_sensor = EV3Sensor(Port.S2)
right_sensor = EV3Sensor(Port.S3)
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE) 
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)
wheelDiameter = 56
wheelBase = 175
car = DriveBase(left_motor, right_motor, wheelDiameter, wheelBase)
#speed = 150
#turn_speed = 60
left_val = left_sensor.read()
right_val = right_sensor.read()
tolerance_range = 50
calibrate_val = 151 # L/R? L
total = 0
run = True
count = 0
watch = StopWatch()

# bang-bang
"""
while run and (count < 1000):
    left_val = left_sensor.read() + calibrate_val
    right_val = right_sensor.read() 
    if (left_val > (right_val + tolerance_range)):
        #turn right
        car.drive(speed, turn_speed)
        #left_motor.run(speed)
        #right_motor.run(-speed)
        print("right")
        pass
        
    elif (left_val < (right_val - tolerance_range)):
        #turn left
        car.drive(speed, -turn_speed)
        #left_motor.run(-speed)
        #right_motor.run(speed)
        print("left")
        pass
        
    else:
        #drive forward
        car.drive(speed, 0)
        #left_motor.run(speed)
        #right_motor.run(speed)
        print("forward")
        pass

    if any (brick.buttons()):
        run = False
  #  total = total + (left_val - right_val)
 #   count = count + 1
    print("left: " + str(left_val) + " right: " + str(right_val))
    #print(" ")
    #wait(1000)

#print(total/1000)
"""

"""
# proportional
# p eq: Pout = k_p * e(t) + p0
# k_p-proportional gain, e(t)-instantaneous process error at time t = SP - PV
# Sp-set point, PV-process variable
speed = 100
desired_r = 200
desired_l = 200
k_p = 2
# 100, 200, 200, 2
#wait(5000)
while run and (count < 1000):
    left_val = left_sensor.read() 
    right_val = right_sensor.read()
    left_speed = (k_p * (left_val - desired_l)) +  speed
    right_speed = (k_p * (right_val - desired_r)) +  speed
    left_motor.run(left_speed)
    right_motor.run(right_speed)
    #diff = left_val - right_val
    #turn_speed = (k_p * (diff - desired))
    #drive_speed = (k_p * -(abs(turn_speed))) + speed
    #car.drive(drive_speed, turn_speed)
    if any (brick.buttons()):
        run = False
    #total = total + (left_val - right_val)
    #count = count + 1
    print("left: " + str(left_val) + " right: " + str(right_val))
    print("left speed: " + str(left_speed) + " right speed: " + str(right_speed))
    #print("left speed: " + str(left_speed) + " right speed: " + str(right_speed))
   # print("turn speed: " + str(turn_speed))
    #wait(10)

# print(total/1000)
"""


# proportional derivative
# p eq: Pout = k_p * e(t) + p0
# k_p-proportional gain, e(t)-instantaneous process error at time t = SP - PV
# Sp-set point, PV-process variable

speed = 0
desired_l = 300
desired_r = 200
k_p = 2
k_d = 2

def prop_deriv_control(val0, val1, t0, t1, desired):
    newspeed_p = k_p * (val1 - desired) + speed
    dt = t1 - t0
    if dt == 0:
        dt =1
    newspeed_d = k_d * ((val1 - val0) / dt)
    return (newspeed_p + newspeed_d)

#wait(5000)
time0 = 0
time1 = 0
L0 = L1 = R0 = R1 = 0
while run and (count < 1000):
    time1 = watch.time()
    if time0 == 0:
        time0 = time1
    L1 = left_sensor.read() + calibrate_val
    R1 = right_sensor.read() 
    if L0 == 0:
        L0 = L1
    if R0 == 0:
        R0 = R1
    left_speed = prop_deriv_control(L0, L1, time0, time1, desired_l)
    right_speed = prop_deriv_control(R0, R1, time0, time1, desired_r)
    time0 = time1
    L0 = L1
    R0 = R1
    left_motor.run(left_speed)
    right_motor.run(right_speed)
    if Button.CENTER in brick.buttons():
        run = False
    print("left: " + str(L1) + " right: " + str(R1))
    print("left speed: " + str(left_speed) + " right speed: " + str(right_speed))
    #total = total + (left_val - right_val)
    #count = count + 1
    #wait(10000)

# print(total/1000)
