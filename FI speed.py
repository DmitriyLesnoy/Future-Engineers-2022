from re import T
import cv2
import RobotAPI as rapi
import numpy as np
import serial
import time


from GPIORobot import GPIORobotApi

robot=GPIORobotApi()

file = open("change.txt", "w")
file.write("0")
file.close()


port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
# robot = rapi.RobotAPI(flag_serial=False)
robot.set_camera(100, 640, 480)

fps = 0
fps1 = 0
fps_time = 0

p=0

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

HSV_black=[[0,0,0],[180,255,70]]

HSV_orange=[[0,20 ,80],[25,255,255]]
HSV_blue=[[100,90,20],[130,255,170]]

HSV_red=[[150,110,70],[180,255,255],[0,70,120],[20,230,240]]
HSV_green=[[60,100,60],[90,255,255],[0,0,0],[0,0,0]]


# ?????????????????????????????????????????????

global_speed=140


states=['start','main','manual','HSV','finish']

porog=0
delta_reg=0
delta_reg_old=0

d_red_o,d_green_o=0,0

delta_banka=0
delta_red=0
delta_green=0
area_green=0
area_red=0
red=False
green=False

flag_min=False

count_green=False
count_red=False

boxes=[0,0]
box_map_count=[[0,0],
               [0,0],
               [0,0],
               [0,0]]
box_map=[[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

map_times=[-1,-1,-1,-1]
zona_times=[-1,-1,-1,-1]

procent_map=[-1,-1,-1,-1]

timer_map=0
flag_timer_map=False

timer_zone=0
flag_zone=False

timer_count=0
flag_count=False
color_box=0

c_line=0
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

flag_wall_r,flag_wall_l=False,False

direction=None

def black_line_left(banka,hsv=HSV_black,hsv_blue=HSV_blue):


    x1,y1=0,270
    x2,y2=20,480

    if banka and direction==-1:
        y1=275+75

    if banka and direction==-1:
        y1=270-10
    # global xb1, yb1, xb2, yb2, lowb, upb, sr, max1


    datb1 = frame[y1:y2,x1:x2]
    cv2.rectangle(frame, (x1, y1),(x2, y2), (255, 255, 255), 2)

    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    maskd2 = cv2.inRange(hsv1,np.array(hsv_blue[0]), np.array(hsv_blue[1]))

    mask=cv2.bitwise_and(cv2.bitwise_not(maskd2),maskd1)

    gray1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    # frame[y1:y2,x1:x2] = gray1

    imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max_y_right = 0

    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor)
        area = cv2.contourArea(contor)
        if area > 200:
            cv2.rectangle(datb1, (x, y), (x + w, y + h), (0, 0, 255), 2)

            if max_y_right < y + h:
                max_y_right = (y + h)/2

        cv2.putText(frame, "" + str(max_y_right), (x1, y1-10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,(0, 0, 255), 2)
    return max_y_right

def black_line_right(banka,hsv=HSV_black,hsv_blue=HSV_blue):

    x1,y1=640-20,270
    x2,y2=640,480
    if banka and direction==1:
        y1=275+75
    if banka and direction==1:
        y1=270-10
    # global xb1, yb1, xb2, yb2, lowb, upb, sr, max1


    datb1 = frame[y1:y2,x1:x2]
    cv2.rectangle(frame, (x1, y1),(x2, y2), (255, 255, 255), 2)

    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    maskd2 = cv2.inRange(hsv1,np.array(hsv_blue[0]), np.array(hsv_blue[1]))

    mask=cv2.bitwise_and(cv2.bitwise_not(maskd2),maskd1)

    gray1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    # frame[y1:y2,x1:x2] = gray1

    imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max_y_right = 0

    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor)
        area = cv2.contourArea(contor)
        if area >200:
            cv2.rectangle(datb1, (x, y), (x + w, y + h), (0, 0, 255), 2)

            if max_y_right < y + h:
                max_y_right = (y + h)/2

        cv2.putText(frame, "" + str(max_y_right), (x1-15,y1-10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,(0, 0, 255), 2)
    return max_y_right

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
        if area > 80:
            cv2.rectangle(datb1, (x, y), (x+w, y+h), (0, 122, 122), 2)
            return True

    return False

def find_wall(hsv=HSV_black):

    x1, y1 = 320-10, 400 
    x2, y2 = 320+10, 440

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

            if area_wall>100:
                flag_wall=True

            if flag_wall:
                cv2.rectangle(datb1, (x, y), (x+w, y+h), (255,0,255), 2)

    return flag_wall

def find_box(hsv,color,hsv_o=HSV_orange):
    x1, y1 = 0, 230
    x2, y2 = 640-0, 480

    datb1 = frame[y1:y2,x1:x2]
    # cv2.rectangle(frame, (x1, y1),(x2, y2), (0, 100, 100), 2)

    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    maskd2 = cv2.inRange(hsv1,np.array(hsv[2]), np.array(hsv[3]))    
    mask_orange = cv2.inRange(hsv1,np.array(hsv_o[0]), np.array(hsv_o[1]))    

    mask=cv2.bitwise_and(cv2.bitwise_or(maskd1,maskd2),cv2.bitwise_not(mask_orange))

    x_banka = 0
    y_banka = 0
    area_banka = 0

    gray1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    # frame[y1:y2,x1:x2] = gray1

    imd1, contours, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    max=0

    x1,y1,w1,h1=0,0,0,0

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 340:
            if y+h>max:
                max=y+h
                x_banka = x + w/2
                y_banka = y + h
                area_banka = area
                x1,y1,w1,h1=x,y,w,h





    cv2.rectangle(datb1, (x1, y1), (x1+w1, y1+h1), (255,255,255), 2)
            # cv2.putText(datb1, str(x_banka)+str(y_    banka), (x, y), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,c1, 2)            # robot.text_to_frame(frame, area_banka, (x + w, y + h), c, 2) 
    # cv2.putText(datb1, str(area_banka), (x1, y1+10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255), 2)            # robot.text_to_frame(frame, area_banka, (x + w, y + h), c, 2) 
    cv2.putText(datb1, str(int(x_banka)) +"-"+ str(int(y_banka)), (x1, y1), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255), 2)            # robot.text_to_frame(frame, area_banka, (x + w, y + h), c, 2) 
    cv2.putText(datb1, color, (x1, y1+15), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,0,0), 2)            # robot.text_to_frame(frame, area_banka, (x + w, y + h), c, 2) 

    return [x_banka, y_banka], area_banka

def count_box(hsv,color,hsv_o=HSV_orange):
    x1, y1 = 0, 365
    x2, y2 = 640, 365+20

    datb1 = frame[y1:y2,x1:x2]
    cv2.rectangle(frame, (x1, y1),(x2, y2), (0, 130, 130), 2)

    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    maskd2 = cv2.inRange(hsv1,np.array(hsv[2]), np.array(hsv[3]))    
    mask_orange = cv2.inRange(hsv1,np.array(hsv_o[0]), np.array(hsv_o[1]))    

    mask=cv2.bitwise_and(cv2.bitwise_or(maskd1,maskd2),cv2.bitwise_not(mask_orange))

    x_banka = None
    y_banka = None
    area_banka = None


    gray1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
    # frame[y1:y2,x1:x2] = gray1

    imd1, contours, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    max=0
    flag=False
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 120:
            if area>max:
                max=area
                x_banka = x + w/2
                y_banka = y + h
                area_banka = area

            if color=='green':
                c = (0, 122, 0) 
            else:
                c= (0,0,122)

            if area_banka>170:
                flag=True
            

            cv2.rectangle(datb1, (x, y), (x+w, y+h), c, 2)

    return flag

def telemetry():
    robot.text_to_frame(frame, 'state = ' + str(state), 10, 20,(255,122,122))
    robot.text_to_frame(frame, 'speed = ' + str(global_speed), 10, 40,(0,0,255))
    robot.text_to_frame(frame, 'serv = ' + str(int(p)), 10, 60,(255,255,0))
    robot.text_to_frame(frame, 'fps = ' + str(fps), 505, 20)
    robot.text_to_frame(frame, 'key = ' + str(k), 505, 40,(122,122,255))


    robot.text_to_frame(frame, 'banka = ' + str(int(delta_banka)), 10, 80,(255,255,255))

    robot.text_to_frame(frame, "FI - Time: " + str(int(secundomer)), 290, 20,(0,0,0)) 

    if direction is not None:
        if direction==1:
            robot.text_to_frame(frame, 'Lin(+)=' + str(count_lines), 485, 60,(0,0,0))
        else:
            robot.text_to_frame(frame, "Lin(-)=" + str(count_lines), 470, 60,(50,50,50))
    
    robot.text_to_frame(frame, "Bs= " + str(boxes), 470, 80,(255,255,255))

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
        robot.light(0,0,0)
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

        is_orange = find_start_line(HSV_orange)
        is_blue = find_start_line(HSV_blue)

        if direction is None:
            if is_orange:
                direction = 1
                robot.tone(180)
            elif is_blue:
                direction = -1

            flag_line = True
            timer_line = time.time()

        else:
            if count_lines<12:
                if is_blue and direction==-1:
                    flag_line=True
                    timer_line=time.time()

                if is_orange and direction==1:
                    flag_line=True
                    timer_line=time.time()
            else:
                if is_blue and direction==1:
                    flag_line=True
                    timer_line=time.time()

                if is_orange and direction==-1:
                    flag_line=True
                    timer_line=time.time() 


            if time.time()>=timer_line+0.125 and flag_line:
                flag_line=False
                count_lines+=1

        if count_lines==1 and timer_zone==0:
            timer_zone=time.time()         
        if count_lines==2:
            if map_times[1]==-1:
                map_times[1]=int((time.time()-timer_zone)*10)/10
                timer_zone=time.time()
        if count_lines==3:
            if map_times[2]==-1:
                map_times[2]=int((time.time()-timer_zone)*10)/10
                timer_zone=time.time()
        if count_lines==4:
            if map_times[3]==-1:
                map_times[3]=int((time.time()-timer_zone)*10)/10
                timer_zone=time.time()     
        if count_lines==5:
            if map_times[0]==-1:
                map_times[0]=int((time.time()-timer_zone)*10)/10
                timer_zone=time.time()

        if int(count_lines) >= 13:
            pause_finish = 0.3*map_times[0]
            if timer_finish is None:
                timer_finish = time.time() + pause_finish

            if timer_finish is not None and time.time()>=timer_finish:
                robot.tone(255)
                robot.tone(130)
                robot.tone(50)
                print(box_map,'b_m')
                # print(map_times,'m_t')
                # print(zona_times,'z_t')
                print(procent_map,'p_z')
                print(int(secundomer*10)/10)
                state='finish'
            robot.light(255,255,255)


        cord_green,area_green=find_box(HSV_green,'green')
        cord_red,area_red=find_box(HSV_red,'red')

        delta_banka=0
        
        p=1.3  # перспектива
        k=-0.15
        d=0.15
        react_area=750

        if area_green is not None and area_green>=react_area:
            e=(round(280 - 5 + cord_green[1]*p) - cord_green[0])
            d_green=e*k+(e-d_green_o)*d
            d_green_o=e
            delta_banka=d_green
            green=True
        else:
            green=False
       
       
        if area_red is not None and area_red>=react_area:
            e=(round(280 + 5 - cord_red[1]*p) - cord_red[0])
            d_red= e*k+(e-d_red_o)*d
            d_red_o=e
            delta_banka=d_red
            red=True
        else:
            red=False


        if red and green:
            if cord_green[1]>cord_red[1]:
                delta_banka= d_green
                red=False
            else:
                delta_banka= d_red
                green=False

        if red or green:
            timer_banka=time.time()
            delta_banka_old=delta_banka
            flag_min=True
        else:
            if time.time()<=timer_banka+0.15:
                delta_banka=0
            if time.time()>=timer_banka+0.55:
                flag_min=False
            if count_lines>=1 and flag_timer_map==False:
                timer_map=int((time.time()-timer_zone)*10)/10
                flag_timer_map=True


        count_green=count_box(HSV_green,'green')
        count_red=count_box(HSV_red,'red')

        if flag_count and not count_green and not count_red and time.time()>=timer_count+0.2:
            flag_count=False
            boxes[c_box-1]+=1


            if count_lines>=1:
                if count_lines==1:
                    box_map_count[1][c_box-1]+=1
                    if box_map[1]==[0,0,0]:
                        box_map[1][0]=c_box
                    else:
                        box_map[1][2]=c_box
                if count_lines==2:
                    box_map_count[2][c_box-1]+=1
                    if box_map[2]==[0,0,0]:
                        box_map[2][0]=c_box
                    else:
                        box_map[2][2]=c_box
                if count_lines==3:
                    box_map_count[3][c_box-1]+=1
                    if box_map[3]==[0,0,0]:
                        box_map[3][0]=c_box
                    else:
                        box_map[3][2]=c_box
                if count_lines==4:
                    box_map_count[0][c_box-1]+=1

                if 2<=count_lines<=4 and flag_timer_map:
                    if box_map_count[int(count_lines)-1]==[2,0]:
                        box_map[int(count_lines)-1]=[1,0,1]
                    if box_map_count[int(count_lines)-1]==[0,2]:
                        box_map[int(count_lines)-1]=[2,0,2]    
                    
                    
                    prcnt=timer_map/map_times[int(count_lines)-1]



                    if count_lines==2:
                        procent_map[1]=int(prcnt*100)/100
                        zona_times[1]=timer_map
                    if count_lines==3:
                        procent_map[2]=int(prcnt*100)/100
                        zona_times[2]=timer_map
                    if count_lines==4:
                        procent_map[3]=int(prcnt*100)/100
                        zona_times[3]=timer_map
                    

                    if prcnt<0.35:
                        if box_map_count[int(count_lines)-1]==[1,0]:
                            box_map[int(count_lines)-1]=[1,0,0]
                        if box_map_count[int(count_lines)-1]==[0,1]:
                            box_map[int(count_lines)-1]=[2,0,0]
                    elif prcnt>0.6:
                        if box_map_count[int(count_lines)-1]==[1,0]:
                            box_map[int(count_lines)-1]=[0,0,1]
                        if box_map_count[int(count_lines)-1]==[0,1]:
                            box_map[int(count_lines)-1]=[0,0,2]
                    else:
                        if box_map_count[int(count_lines)-1]==[1,0]:
                            box_map[int(count_lines)-1]=[0,1,0]
                        if box_map_count[int(count_lines)-1]==[0,1]:
                            box_map[int(count_lines)-1]=[0,2,0]

                if count_lines==4:
                    if box_map_count[0]==[2,0]:
                        box_map[0]=[1,0,1]
                    if box_map_count[0]==[0,2]:
                        box_map[0]=[2,0,2] 

                if box_map[0]==[0,0,0] and direction is not None:
                    if direction==1:
                        box_map[0]=[0,2,0]
                    if direction==-1:
                        box_map[0]=[0,1,0]

                flag_timer_map=False            

        if state=='finish':
            if box_map_count[0]==[1,0]:
                boxes[0]+=1
            if box_map_count[0]==[0,1]:
                boxes[1]+=1

        if count_green:
            timer_count=time.time()
            flag_count=True
            c_box=1
        
        if count_red:
            timer_count=time.time()
            flag_count=True
            c_box=2




        max_l=black_line_left(flag_min)
        max_r=black_line_right(flag_min)

        porog=0  # 10
        
        delta_reg = max_l - max_r + porog

        p = int(delta_reg * 0.2 + (delta_reg - delta_reg_old) * 0.2)  # 0.8 0.9
        delta_reg_old = delta_reg


        if max_r==0:
            p=40
        if max_l==0:
            p=-40
        if max_l+max_r==0 and direction is not None:
            p=60*direction
        if find_wall():
            p=60*direction
            if flag_line==True:
                flag_line=False

        if red or green:
            p=delta_banka


        if red:
            robot.light(255,0,0)
        elif green:
            robot.light(0,255,0)
        elif flag_line:
            if direction is not None:
                if direction==1:
                    robot.light(255,35,0)
                if direction==-1:
                    robot.light(0,0,255)
        else:
            robot.light(0,0,0)

        robot.serv(-p)

        if global_speed<=0:
            global_speed=0
        if global_speed>=255:
            global_speed=255
        robot.move(global_speed)   
    if state=='finish':
        robot.light(255,122,122)
        robot.serv(0)
        robot.move(15,False)

    if state=='manual':
        if k==187:
            global_speed+=1
        if k==189:
            global_speed-=1
        if k==65:
            nup=60
        if k==68:
            nup=-60
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