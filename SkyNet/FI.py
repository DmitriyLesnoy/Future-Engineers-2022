import cv2
from cv2 import FILE_STORAGE_FORMAT_JSON
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

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

HSV_black=[[0,0,0],[180,255,60]]

# ?????????????????????????????????????????????

global_speed=0

porog=0
delta_reg=0
delta_reg_old=0

def black_line_left(hsv):

    x1,y1=0,300
    x2,y2=20,480

    global xb1, yb1, xb2, yb2, lowb, upb, sr, max1


    datb1 = frame[y1:y2,x1:x2]
    cv2.rectangle(frame, (x1, y1),(x2, y2), (255, 255, 255), 2)

    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))

    gray1 = cv2.cvtColor(maskd1, cv2.COLOR_GRAY2BGR)
    # frame[y1:y2,x1:x2] = gray1

    imd1, contours, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max_y_right = 0

    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor)
        area = cv2.contourArea(contor)
        if area > 200:
            cv2.rectangle(datb1, (x, y), (x + w, y + h), (0, 0, 255), 2)

            if max_y_right < y + h:
                max_y_right = y + h

        cv2.putText(frame, "" + str(max_y_right), (x1, y1-10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                    (0, 0, 255), 2)
    return max_y_right

def black_line_right(hsv):

    x1,y1=640-20,300
    x2,y2=640,480

    global xb1, yb1, xb2, yb2, lowb, upb, sr, max1


    datb1 = frame[y1:y2,x1:x2]
    cv2.rectangle(frame, (x1, y1),(x2, y2), (255, 255, 255), 2)

    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))

    gray1 = cv2.cvtColor(maskd1, cv2.COLOR_GRAY2BGR)
    # frame[y1:y2,x1:x2] = gray1

    imd1, contours, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max_y_right = 0

    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor)
        area = cv2.contourArea(contor)
        if area >200:
            cv2.rectangle(datb1, (x, y), (x + w, y + h), (0, 0, 255), 2)

            if max_y_right < y + h:
                max_y_right = y + h

        cv2.putText(frame, "" + str(max_y_right), (x1-15,y1-10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                    (0, 0, 255), 2)
    return max_y_right

nup=0
mv=0

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
    

    # if k==49:
    #     nup=25
    #     print(nup)
    # if k==51:
    #     nup=-25
    #     print(nup)
    # if k==50:
    #     nup=0
    #     print(nup)
    # if k==52:
    #     robot.move(100, True)
    #     time.sleep(0.15)
    #     # robot.move(0, True)
    # if k==53:
    #     robot.move(100, False)
    #     time.sleep(0.15)
    #     # robot.move(0, False)
    # else:
    #     robot.move(0, True)

    # if nup>=25:
    #     nup=25
    # if nup<=-25:
    #     nup=-25
    # robot.serv(angle=nup)

    if k==49:
        global_speed+=5
    if k==50:
        global_speed-=5
    if k!=-1:
        print(global_speed)

    if global_speed<=0:
        global_speed=0
    # if global_speed>=70:
    #     global_speed=70

    max_l=black_line_left(HSV_black)
    max_r=black_line_right(HSV_black)

    delta_reg = max_l - max_r + porog

    p = int(delta_reg * 0.5 + (delta_reg - delta_reg_old) * 0.6)
    delta_reg_old = delta_reg

    if max_r==0:
        p=18
    if max_l==0:
        p=-18


    if p>=25:
        p=25
    if p<=-25:
        p=25

    robot.serv(-p)

    robot.move(global_speed)   

    robot.text_to_frame(frame, 'serv = ' + str(p), 10, 20,(0,255,0))


    robot.text_to_frame(frame, 'fps = ' + str(fps), 500, 20)
    robot.set_frame(frame, 40)