import cv2
import RobotAPI as rapi
import numpy as np
import serial
import time


from GPIORobot import GPIORobotApi

robot=GPIORobotApi()


port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
# robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

fps = 0
fps1 = 0
fps_time = 0


# def black_line():
#     global xb1, yb1, xb2, yb2, lowb, upb, sr, max1
#     datb1 = frame[yb1:yb2, xb1:xb2]
#     cv2.rectangle(frame, (xb1, yb1), (xb2, yb2), (0, 0, 255), 2)

#     dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
#     hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
#     maskd1 = cv2.inRange(hsv1, lowb, upb)

#     gray1 = cv2.cvtColor(maskd1, cv2.COLOR_GRAY2BGR)
#     frame[yb1:yb2, xb1:xb2] = gray1

#     imd1, contoursd1, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
#     max1 = 0

#     for contorb1 in contoursd1:
#         x, y, w, h = cv2.boundingRect(contorb1)
#         a1 = cv2.contourArea(contorb1)
#         if a1 > 1000:
#             if a1 > max1:
#                 max1 = a1
#                 sr = (x + x + w)//2
#             cv2.rectangle(datb1, (x, y), (x + w, y + h), (0, 255, 0), 2)

nup=0
while 1:

    frame = robot.get_frame(wait_new_frame=1)

    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0

    k=robot.get_key()
    if k!=-1:
      print(k)

    if k==49:
        nup+=10
        print(nup)
    if k==50:
        nup -= 10
        print(nup)
    if k==51:
        nup=0
        print(nup)

    robot.serv(angle=nup)

    # robot.serv(-25)
    # time.sleep(1)
    # robot.serv(0)
    # time.sleep(1)
    # robot.serv(25)
    # time.sleep(1)
    # robot.serv(0)
    # time.sleep(1)

    # robot.move(40,False)
    # time.sleep(1)
    # robot.move(0, False)
    # time.sleep(1)

    



    robot.text_to_frame(frame, 'fps = ' + str(fps), 500, 20)
    robot.set_frame(frame, 40)