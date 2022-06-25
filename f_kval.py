import cv2
import RobotAPI as rapi
import numpy as np
import serial
import time


from GPIORobot import GPIORobotApi

robot=GPIORobotApi()

file = open("change.txt", "w")
file.write("1")
file.close()

port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
# robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

fps = 0
fps1 = 0
fps_time = 0

p=0
side_reg=-15

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

HSV_black=[[0,0,0],[180,255,70]]

HSV_orange=[[0,20 ,80],[25,255,255]]
HSV_blue=[[100,90,20],[130,255,170]]


# ?????????????????????????????????????????????

global_speed=255



states=['start','main','manual','HSV','finish']

porog=0
delta_reg=0
delta_reg_old=0

count_lines=0

timer_finish = None

timer_sec=None
secundomer=0

direction=None

def black_line_left(hsv,hsv_blue=HSV_blue):


    x1,y1=0,315
    x2,y2=250,345 


    # global xb1, yb1, xb2, yb2, lowb, upb, sr, max1


    datb1 = frame[y1:y2,x1:x2]
    cv2.rectangle(frame, (x1, y1),(x2, y2), (255, 255, 255), 2)

    # dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    
    hsv1 = cv2.cvtColor(datb1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    maskd1 = cv2.blur(maskd1,(3,3))

    maskd2 = cv2.inRange(hsv1,np.array(hsv_blue[0]), np.array(hsv_blue[1]))
    maskd2 = cv2.blur(maskd2,(3,3))

    mask=cv2.bitwise_and(cv2.bitwise_not(maskd2),maskd1)

    gray1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    # frame[y1:y2,x1:x2] = gray1

    imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max_s_left = 0
    max=0
    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor)
        area = cv2.contourArea(contor)
        if area > 200:
            

            if max < w*h and area>h*w*0.3 and h>10:
                max_s_left = (x+w)*h
                max=h*w
                cv2.rectangle(datb1, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(frame, "" + str(max_s_left), (x1, y1-10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                    (255, 0, 0), 2)
    return max_s_left

def black_line_right(hsv,hsv_blue=HSV_blue):

    x1,y1=640-250,315
    x2,y2=640,345 

    # global xb1, yb1, xb2, yb2, lowb, upb, sr  max1


    datb1 = frame[y1:y2,x1:x2]
    cv2.rectangle(frame, (x1, y1),(x2, y2), (255, 255, 255), 2)

    # dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)

    hsv1 = cv2.cvtColor(datb1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    maskd1=cv2.blur(maskd1,(3,3))

    maskd2 = cv2.inRange(hsv1,np.array(hsv_blue[0]), np.array(hsv_blue[1]))
    maskd2=cv2.blur(maskd2,(3,3))

    mask=cv2.bitwise_and(cv2.bitwise_not(maskd2),maskd1)


    gray1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    # frame[y1:y2,x1:x2] = gray1

    imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max_s_right = 0
    max=0
    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor)
        area = cv2.contourArea(contor)
        if area >200:

            if max < w*h and area>h*w*0.3 and h>10:
                max=h*w
                max_s_right = ((640-x1)-x)*h
                cv2.rectangle(datb1, (x, y), (x + w, y + h), (0, 0, 255), 2)

        cv2.putText(frame, "" + str(max_s_right), (x1-15,y1-10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                    (255, 0, 0), 2)
    return max_s_right

def find_start_line(hsv):
    x1, y1 = 320 - 20, 440
    x2, y2 = 320 + 20, 480

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
        if area > 70:
            cv2.rectangle(datb1, (x, y), (x+w, y+h), (0, 122, 122), 2)
            return True

    return False

def find_wall(hsv):
    x1, y1 = 320-20, 320
    x2, y2 = 320+20, 350

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

            if area_wall>420:
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
    robot.text_to_frame(frame, "Kval-Time: " + str(int(secundomer)), 290, 20,(0,0,0)) 
    if direction is not None:
        if direction==1:
            robot.text_to_frame(frame, 'Lines(+) = ' + str(count_lines), 10, 80,(0,0,0))
        if direction==-1:
            robot.text_to_frame(frame, "Lines(-) = " + str(count_lines), 10, 80,(0,0,0))

nup=0
mv=0

robot.move(0)
robot.serv(-60)
robot.serv(60)
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
            robot.move(0)
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

        is_orange = find_start_line(HSV_orange)
        is_blue = find_start_line(HSV_blue)

        if direction==None:
            if is_orange:
                direction = 1
                robot.tone(170)
                time.sleep(0.1)
            elif is_blue:
                direction = -1

            flag_line = True
            timer_line = time.time()

        else:
            if count_lines<=10:
                if direction==-1 and is_blue:
                    flag_line=True
                    timer_line=time.time()

                if direction==1 and is_orange:
                    flag_line=True
                    timer_line=time.time()
            else:
                robot.light(255,255,255)
                if direction==-1 and is_orange:
                    flag_line=True
                    timer_line=time.time()

                if direction==1 and is_blue:
                    flag_line=True
                    timer_line=time.time()

            if time.time()>=timer_line+0.7 and flag_line:
                flag_line=False
                count_lines+=1

        if count_lines >= 12:
            pause_finish = 0
            if timer_finish is None:
                timer_finish = time.time() + pause_finish

            if timer_finish is not None and time.time()>=timer_finish:
                robot.tone(255)
                state='finish'



        max_l=black_line_left(HSV_black)
        max_r=black_line_right(HSV_black)

        delta_reg = max_l - max_r

        if -50<delta_reg<50:
            delta_reg=0

        delta_reg=delta_reg//50

        p = int(delta_reg * 0.2 + (delta_reg - delta_reg_old) * 0.3)
        delta_reg_old = delta_reg

        if max_l+max_r==0 and direction is not None:
            p=30*direction

        else:
            if max_l==0:
                p=-30
            if max_r==0:
                p=30

        robot.serv(-p)

        if global_speed<=0:
            global_speed=0
        if global_speed>=255:
            global_speed=255
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
            nup=40
        if k==68:
            nup=-40
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
