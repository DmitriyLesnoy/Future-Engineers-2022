import cv2
import RobotAPI as rapi
import numpy as np
import serial
import time

from GPIORobot import GPIORobotApi

Grobot=GPIORobotApi()

port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

fps = 0
fps_count = 0
t = time.time()

while 1:
    fps_count += 1
    if time.time() > t + 1:
        fps = fps_count
        fps_count = 0
        t = time.time()
    
    k = robot.get_key()
    if k == 87:
        robot.move(50)
    elif k == 83:
        robot.move(50, False)
    elif k == 32:
        robot.move(0)
    if k == 65:
        robot.serv(-30)
    elif k == 68:
        robot.serv(30)

    frame = robot.get_frame(wait_new_frame=1)
    # frame = cv2.flip(frame, 1
    robot.text_to_frame(frame, "FPS: " + str(fps), 20, 20)
    robot.set_frame(frame, 40)

