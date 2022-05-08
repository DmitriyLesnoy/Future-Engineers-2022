import cv2
from cv2 import FILE_STORAGE_FORMAT_JSON
from cv2 import FILE_STORAGE_WRITE_BASE64
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

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

HSV_black=[[6,17,0],[100,230,70]]

HSV_orange=[[0,28,85],[63,133,255]]
HSV_blue=[[100,50,50],[170,210,200]]

HSV_red=[[119,110,48],[180,255,255],[0,108,76],[14,182,255]]
HSV_green=[[70,77,73],[90,255,189],[0,0,0],[0,0,0]]


# ?????????????????????????????????????????????

global_speed=32


states=['start','main','manual','HSV','finish']

porog=0
delta_reg=0
delta_reg_old=0

delta_banka=0
delta_red=0
delta_green=0
area_green=0
area_red=0

count_lines=0
timer_line=0
flag_line=False

timer_finish = None
pause_finish = 0.9

timer_sec=None
secundomer=0

timer_banka=0
flag_banka=False
delta_banka_old=0

direction=0

def black_line_left(hsv=HSV_black):

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

def black_line_right(hsv=HSV_black):

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

def find_wall(input,hsv=HSV_black):
    x1, y1 = 320-7, 230
    x2, y2 = 320+7, 350

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
        if area > 200:
            if area>max:
                max=area

                area_wall = area

            if area_wall>input:
                flag_wall=True

            if flag_wall:
                cv2.rectangle(datb1, (x, y), (x+w, y+h), (255,0,255), 2)

    return flag_wall

def find_box(hsv,color):
    x1, y1 = 0, 235
    x2, y2 = 640, 400

    datb1 = frame[y1:y2,x1:x2]
    cv2.rectangle(frame, (x1, y1),(x2, y2), (0, 100, 100), 2)

    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    maskd2 = cv2.inRange(hsv1,np.array(hsv[2]), np.array(hsv[3]))

    mask=maskd1+maskd2

    x_banka = None
    y_banka = None
    area_banka = None


    gray1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    # frame[y1:y2,x1:x2] = gray1

    imd1, contours, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    max=0

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 400:
            if area>max:
                max=area
                x_banka = x + w/2
                y_banka = y + h
                area_banka = area

            if color=='green':
                c = (0, 255, 0)
                c1=(75,255,75)
            else:
                c= (0,0,255)
                c1=(75,75,200)


            cv2.rectangle(datb1, (x, y), (x+w, y+h), c, 2)
            # cv2.putText(datb1, str(x_banka)+str(y_    banka), (x, y), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,c1, 2)            # robot.text_to_frame(frame, area_banka, (x + w, y + h), c, 2) 
            cv2.putText(datb1, str(area_banka), (x, y), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,c1, 2)            # robot.text_to_frame(frame, area_banka, (x + w, y + h), c, 2) 
            cv2.putText(datb1, str(int(x_banka)) +"-"+ str(int(y_banka)), (x, y-20), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,c1, 2)            # robot.text_to_frame(frame, area_banka, (x + w, y + h), c, 2) 

    return x_banka, y_banka, area_banka

def find_ungle(input,hsv=HSV_black):
    x1, y1 = 40, 300
    x2, y2 = 150, 480

    datb1 = frame[y1:y2,x1:x2]
    # cv2.rectangle(frame, (x1, y1),(x2, y2), (150, 150, 150), 2)

    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))

    area_wall = None
    flag_ungle=False

    gray1 = cv2.cvtColor(maskd1, cv2.COLOR_GRAY2BGR)
    # frame[y1:y2,x1:x2] = gray1

    imd1, contours, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    max=0

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 200:
            if area>max:
                max=area

                area_wall = area

            if area_wall>input:
                flag_ungle=True

            if flag_ungle:
                cv2.rectangle(datb1, (x, y), (x+w, y+h), (255,255,255), 2)

    return flag_ungle

def telemetry():
    robot.text_to_frame(frame, 'state = ' + str(state), 10, 20,(255,122,122))
    robot.text_to_frame(frame, 'speed = ' + str(global_speed), 10, 40,(0,0,255))
    robot.text_to_frame(frame, 'serv = ' + str(int(p)), 10, 60,(255,255,0))
    robot.text_to_frame(frame, 'fps = ' + str(fps), 505, 20)
    robot.text_to_frame(frame, 'key = ' + str(k), 505, 40,(122,122,255))
    if area_green is None and area_red is None:
        d=(255,255,255)

    if area_green is not None:
        d=(0,255,0)
    if area_red is not None:
        d=(0,0,255)
    if area_green is not None and area_red is not None:
        if area_green>area_red:
            d=(0,255,0)
        else:
            d=(0,0,255)

    robot.text_to_frame(frame, 'banka = ' + str(int(delta_banka)), 10, 80,d)

    robot.text_to_frame(frame, "Time: " + str(int(secundomer)), 290, 20,(0,0,0)) 
    if direction!=0:
        if direction==1:
            robot.text_to_frame(frame, 'Lin(+)= ' + str(count_lines), 485, 60,(0,0,0))
        else:
            robot.text_to_frame(frame, "Lin(-)= " + str(count_lines), 470, 60,(50,50,50))

nup=0
mv=0

robot.serv(-20)
robot.serv(20)
robot.serv(0)
time.sleep(0.15)

robot.tone(120)
time.sleep(0.15)
robot.tone(150)
time.sleep(0.15)
robot.tone(180)
time.sleep(0.15)
robot.tone(210)

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
        robot.move(0)   
        robot.serv(0)
        if robot.button()==0 or k==50:
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

            if time.time()>=timer_line+0.7 and flag_line:
                flag_line=False
                count_lines+=1

        if count_lines >= 12:
            pause_finish = 0.3
            if timer_finish is None:
                timer_finish = time.time() + pause_finish
            if time.time()>=timer_finish:
                robot.serv(0)
                robot.move(20,False)
                time.sleep(0.1)
                robot.move(0)
                robot.tone(120)
                robot.tone(50)
                robot.tone(255)
                state='finish'



        x_green,y_green,area_green=find_box(HSV_green,'green')
        x_red,y_red,area_red=find_box(HSV_red,'red')

        delta_banka=0
        k=3  # 4 - чем больше k тем меньше отворот по x
        r=1.6 # 1.2 - чем больше r тем больше отворот по близости (перспектива)
        react_area=750

        if area_green is not None and area_green>=react_area:
            if x_green < 320:
                delta_green = -25
            elif x_green > 600:
                delta_green = 0
            else:
                delta_green = 0 - (640-160+(y_green-115)*r - x_green) / k
            delta_banka=delta_green
        if area_red is not None and area_red>=react_area:
            if x_red > 320:
                delta_red = 25
            elif x_red < 40:
                delta_red = 0
            else:
                delta_red = 0 + (x_red - (160-(y_red-115)*r)) / k
            delta_banka=delta_red

        if area_green is not None and area_red is not None:
            if y_green>y_red:
                delta_banka= delta_green
            else:
                delta_banka= delta_red

        if area_green is not None or area_red is not None:
            timer_banka=time.time()
            flag_banka=True
            delta_banka_old=delta_banka
        else:
            if time.time()<=timer_banka+0.3 and flag_banka:
                robot.light(0,255,255)
                delta_banka=delta_banka_old
            else:
                flag_banka=False
                robot.light(0,0,0)

        max_l=black_line_left(HSV_black)
        max_r=black_line_right(HSV_black)

        porog=15

        delta_reg = max_l - max_r + porog

        p = int(delta_reg * 0.7 + (delta_reg - delta_reg_old) * 0.7)
        delta_reg_old = delta_reg


        if max_r==0:
            p=16
        if max_l==0:
            p=-16


        if delta_banka!=0:
            p=delta_banka

        if p>=25:
            p=25
        if p<=-25:
            p=-25

        flag_wall=find_wall(400)

        if (flag_wall and delta_banka==0) or (max_r==0 and max_l==0 and delta_banka==0) :
            if direction==1 and max_r==0:
                p=25
            if direction==-1 and max_l==0:
                p=-25

        robot.serv(-p)

        if global_speed<=0:
            global_speed=0
        robot.move(global_speed)   

    if state=='finish':
        robot.serv(0)
        robot.move(10,False)


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
            robot.move(global_speed, True
            )
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