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

p=0
side_reg=0

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

HSV_black=[[6,17,0],[100,230,70]]
HSV_orange=[[0,62,115],[12,221,255]]
HSV_blue=[[100,70,0],[170,255,255]]

9
# ?????????????????????????????????????????????

global_speed=32


states=['start','main','manual','HSV','finish']

porog=0
delta_reg=0
delta_reg_old=0

count_lines=0

timer_finish = None
pause_finish = 0.9

timer_sec=None
secundomer=0

direction=None

def black_line_left(hsv):

    x1,y1=0,280
    x2,y2=20,480

    # global xb1, yb1, xb2, yb2, lowb, upb, sr, max1


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
        if area > 100:
            cv2.rectangle(datb1, (x, y), (x + w, y + h), (0, 0, 255), 2)

            if max_y_right < y + h:
                max_y_right = (y + h)/2

        cv2.putText(frame, "" + str(max_y_right), (x1, y1-10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                    (0, 0, 255), 2)
    return max_y_right

def black_line_right(hsv):

    x1,y1=640-20,280
    x2,y2=640,480

    # global xb1, yb1, xb2, yb2, lowb, upb, sr, max1


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
        if area >100:
            cv2.rectangle(datb1, (x, y), (x + w, y + h), (0, 0, 255), 2)

            if max_y_right < y + h:
                max_y_right = (y + h)/2

        cv2.putText(frame, "" + str(max_y_right), (x1-15,y1-10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                    (0, 0, 255), 2)
    return max_y_right

def find_start_line(hsv):
    x1, y1 = 340 - 20, 440
    x2, y2 = 340 + 20, 480

    # global xb1, yb1, xb2, yb2, lowb, upb, sr, max1


    datb1 = frame[y1:y2,x1:x2]
    cv2.rectangle(frame, (x1, y1),(x2, y2), (255, 255, 255), 2)

    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))

    gray1 = cv2.cvtColor(maskd1, cv2.COLOR_GRAY2BGR)
    # frame[y1:y2,x1:x2] = gray1

    imd1, contours, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 80:
            cv2.rectangle(datb1, (x, y), (x+w, y+h), (0, 122, 122), 2)
            return True

    return False

def find_wall(hsv):
    x1, y1 = 320-20, 300
    x2, y2 = 320+20, 330

    datb1 = frame[y1:y2,x1:x2]
    cv2.rectangle(frame, (x1, y1),(x2, y2), (150, 150, 150), 2)

    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))

    area_wall = None
    flag_wall=False

    gray1 = cv2.cvtColor(maskd1, cv2.COLOR_GRAY2BGR)
    # frame[y1:y2,x1:x2] = gray1

    imd1, contours, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    max=0

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 100:
            if area>max:
                max=area

                area_wall = area

            if area_wall>350:
                flag_wall=True

            if flag_wall:
                cv2.rectangle(datb1, (x, y), (x+w, y+h), (255,0,255), 2)

    return flag_wall

def telemetry():
    robot.text_to_frame(frame, 'state = ' + str(state), 10, 20,(255,122,122))
    robot.text_to_frame(frame, 'speed = ' + str(global_speed), 10, 40,(0,0,255))
    robot.text_to_frame(frame, 'serv = ' + str(p), 10, 60,(0,255,0))
    robot.text_to_frame(frame, 'fps = ' + str(fps), 510, 20)
    robot.text_to_frame(frame, 'key = ' + str(k), 510, 40,(122,122,255))
    robot.text_to_frame(frame, "Time: " + str(int(secundomer)), 290, 20,(0,0,0)) 
    if direction!=None:
        if dir==1:
            robot.text_to_frame(frame, 'Lines(+) = ' + str(count_lines), 10, 80,(0,0,0))
        else:
            robot.text_to_frame(frame, "Lines(-) = " + str(count_lines), 10, 80,(0,0,0))

nup=0
mv=0

robot.serv(-20)
robot.serv(20)
robot.serv(0)
time.sleep(0.15)

robot.tone(120)
robot.light(255,0,0)
time.sleep(0.15)

robot.tone(150)
robot.light(0,255,0)

time.sleep(0.15)

robot.tone(180)
robot.light(0,0,255)

time.sleep(0.15)

robot.tone(210)
robot.light(255,255,255)

state='start'


while 1:

    frame = robot.get_frame(wait_new_frame=1)

    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0

    k=robot.get_key()

    if k==49:
        state='start'
    if k==50:
        state='main'
    if k==51:
        state='manual'
    if k==52:
        state='HSV'

    if state=='start':
        if robot.button()==0 or k==50:
            robot.light(0,0,0)
            robot.move(5)
            robot.serv(0)
            state='main'
        else:
            state='start'
    
    if state=='main':

        if timer_sec==None:
            timer_sec=time.time()
        secundomer=time.time()-timer_sec

        if k==187:
            global_speed+=1
        if k==189:
            global_speed-=1
        if k==66:
            robot.tone(200)

        is_orange = find_start_line(HSV_blue)
        is_blue = find_start_line(HSV_orange)

        if direction==None:
            if is_orange:
                direction = -1
            elif is_blue:
                direction = 1

            flag_line = True
            timer_line = time.time()

        else:
            if direction==-1 and is_blue:
                flag_line=True
                timer_line=time.time()

            if direction==1 and is_orange:
                flag_line=True
                timer_line=time.time()

            if time.time()>=timer_line+0.5 and flag_line:
                flag_line=False
                count_lines+=1

        if count_lines >= 12:
            pause_finish = 30 / global_speed-0.5
            # if time.time()>timer_state+1:
            if timer_finish is None:
                timer_finish = time.time() + pause_finish
                robot.serv(0)
                robot.move(20,False)
                time.sleep(0.1)
                robot.move(0)
                robot.tone(120)
                robot.tone(50)
                robot.tone(255)
                state='finish'



        max_l=black_line_left(HSV_black)
        max_r=black_line_right(HSV_black)

        delta_reg = max_l - max_r + porog

        if max_l>0 and max_r>0:
            if max_l+max_r>145:
                side_reg=145
            else:
                side_reg=95

        if max_r>side_reg:
            max_r=side_reg
        if max_l>side_reg:
            max_l=side_reg

        p = int(delta_reg * 0.5 + (delta_reg - delta_reg_old) * 0.6)
        delta_reg_old = delta_reg

        robot.light(0,0,0)
        if max_r==0:
            p=16
            robot.light(0,255,0)

        if max_l==0:
            p=-16
            robot.light(0,0,255)

        if find_wall(HSV_black):
            robot.light(255,0,0)

            if direction==1:
                p=25
            if direction==-1:
                p=-25
            else:
                pass
                    


        

        if p>=25:
            p=25
        if p<=-25:
            p=-25

        # p=0

        robot.serv(-p)

        if global_speed<=0:
            global_speed=0
        robot.move(global_speed)   

    if state=='finish':
        robot.serv(0)
        robot.move(15,False)


    if state=='manual':
        if k==187:
            global_speed+=1
        if k==189:
            global_speed-=1
        if k==65:
            nup=25
        if k==68:
            nup=-25
        if k==83:
            nup=0
        if k==87:
            robot.move(global_speed, True)
            time.sleep(0.15)
        if k==88:
            robot.move(global_speed, False)
            time.sleep(0.15)
        else:
            robot.move(0, True)
        robot.serv(nup)

        if k==66:
            robot.tone(200)

        if robot.button()==0:
            robot.tone(120)
        if k==56:
            robot.light(255,0,0)
        if k==57:
            robot.light(0,255,0)
        if k==48:
            robot.light(0,0,255)
        if k==55:
            robot.light(255,255,255)  
    telemetry()
    robot.set_frame(frame, 40)
