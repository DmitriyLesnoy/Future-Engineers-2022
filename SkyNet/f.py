import cv2
import RobotAPI as rapi
import numpy as np
import serial
import pigpio as pi
import time

import time


from GPIORobot import GPIORobotApi

robot=GPIORobotApi()


port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
# robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

fps = 0
fps1 = 0
fps_time = 0

HSV_black=[[0,62,0],[180,213,50]]

nup=0
mv=0

PIN=26
pi.set_mode(PIN, pigpio.OUTPUT)


while 1:

    frame = robot.get_frame(wait_new_frame=1)

    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0

    k=robot.get_key()
    # if k!=-1:   
    #   print(k)

    if k==49:
        # pi.set_PWM_frequency(PIN, 2700)
 
        pi.set_PWM_dutycycle(PIN, 128)
        
        time.sleep(0.2)
        
        pi.set_PWM_dutycycle(PIN, 0)
        
        pi.stop()

    robot.text_to_frame(frame, 'fps = ' + str(fps), 500, 20)
    robot.set_frame(frame, 40)