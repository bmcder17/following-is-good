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
utime.sleep(1)
s3 = LegoPort(address ='ev3-ports:in3')
s3.mode = 'ev3-analog'
utime.sleep(1)

# Write your program here
brick.sound.beep()

# testing
# sensor=EV3Sensor(Port.S1) # same port as above
# while True:
#    print(sensor.read())
#    wait(2000)

# global object declarations
left_sensor = EV3Sensor(Port.S2)
right_sensor = EV3Sensor(Port.S3)
left_motor = Motor(Port.A, Direction.COUNTERCLOCKWISE) 
right_motor = Motor(Port.D, Direction.COUNTERCLOCKWISE)

# bang-bang controller
def bang_bang(calibrate_val):
    #initialize local variables
    wheelDiameter = 56
    wheelBase = 175
    car = DriveBase(left_motor, right_motor, wheelDiameter, wheelBase)  # used the DriveBase class in this controller for convenience
    left_val = left_sensor.read()
    right_val = right_sensor.read()
    run = True
    speed = 200        # values chosen from testing to be the best for our robot
    turn_speed = 120
    tolerance_range = 100   # Effectively controls how sensitive the controller is to changes in the light signal

    while run:
        left_val = left_sensor.read() + calibrate_val   # left sensor was determined through testing to need a static offeset to match the right sensor's value
        right_val = right_sensor.read() 

        # decides to turn based on the difference between sensor values 
        if (left_val > (right_val + tolerance_range)):
            # turn right
            car.drive(speed, turn_speed)
        elif (left_val < (right_val - tolerance_range)):
            # turn left
            car.drive(speed, -turn_speed)
        else:
            # drive forward
            car.drive(speed, 0)

        if Button.CENTER in brick.buttons():
            # ends run loop
            run = False

    # ends main loop
    return True 

# proportional controller
def proportional(calibrate_val):
    # initialize local variables
    left_val = left_sensor.read()
    right_val = right_sensor.read()
    run = True
    speed = 100     # values determined from testing to be the best four our robot
    desired_r = 200 # desired right sensor value
    desired_l = 200 # desired left sensor value
    k_p = 2         # proportional constant

    while run and (count < 1000):
        left_val = left_sensor.read() + calibrate_val   # left sensor was determined through testing to need a static offeset to match the right sensor's value
        right_val = right_sensor.read()
        left_speed = (k_p * (left_val - desired_l)) +  speed    # left side transfer function from the sensor output to the motor input
        right_speed = (k_p * (right_val - desired_r)) +  speed  # right side transfer function from the sensor output to the motor input
        left_motor.run(left_speed)
        right_motor.run(right_speed)

        if Button.CENTER in brick.buttons():
            # ends run loop
            run = False
    
    # ends main loop
    return True
    


# proportional-derivative transfer function, this function was created to clear some of the clutter from the controller, this function also works with any sensor
def prop_deriv_control(val0, val1, t0, t1, desired, k_p, k_d, speed):
    newspeed_p = k_p * (val1 - desired) + speed
    dt = t1 - t0    
    if dt == 0:
        dt =1
    newspeed_d = k_d * ((val1 - val0) / dt)
    return (newspeed_p + newspeed_d)

# propotional-derivative controller
def proportional_derivative(calibrate_val):
    # initialize local variables
    watch = StopWatch()
    prev_time = 0       # time when the previous value was taken
    curr_time = 0       # time when the current value was taken
    prev_left = 0       # previous left sensor value
    prev_right = 0      # previous right sensor value
    curr_left = 0       # current left sensor value
    curr_right = 0      # current right sensor value
    run = True

    speed = 0           # values determined from testing to be the best four our robot
    desired_r = 200     # desired right sensor value
    desired_l = 200     # desired left sensor value
    k_p = 2             # proportional constant
    k_d = 2             # derivative constant
    while run and (count < 1000):
        curr_time = watch.time()
        if curr_time == 0:    
            prev_time = curr_time
        curr_left = left_sensor.read() + calibrate_val   # left sensor was determined through testing to need a static offeset to match the right sensor's value
        curr_right = right_sensor.read() 
        if prev_left == 0:
            prev_left = curr_left
        if prev_right == 0:
            prev_right = curr_right
        left_speed = prop_deriv_control(L0, L1, time0, time1, desired_l, k_p, k_d, speed)
        right_speed = prop_deriv_control(R0, R1, time0, time1, desired_r, k_p, k_d, speed)
        prev_time = curr_time
        prev_left = curr_left
        prev_right = curr_right
        left_motor.run(left_speed)
        right_motor.run(right_speed)
        if Button.CENTER in brick.buttons():
            # ends run loop
            run = False
    
    # ends main loop
    return True
  
# initialize local variable
calibrate = 114 # add to the left sensor
end = False

# main loop
while not end:
    if Button.LEFT in brick.buttons():
        end = bang_bang(calibrate)
    elif Button.UP in brick.buttons():
        end = proportional(calibrate)
    elif Button.RIGHT in brick.buttons():
        end = proportional_derivative(calibrate)
