import cv2
import RobotAPI as rapi
import numpy as np
import serial
import time
#импорт необходимых библиотек


from GPIORobot import GPIORobotApi

robot=GPIORobotApi()
#инициализация объекта управлени роботом

port = serial.Serial("/dev/ttyS0", baudrate=115200, stopbits=serial.STOPBITS_ONE)
robot.set_camera(100, 640, 480)
# инициализация камеры 

fps = 0
fps1 = 0
fps_time = 0
# определение переменных ля счета кадров в секунду

p=0

# !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!

HSV_black=[[0,0,0],[180,255,70]]

HSV_orange=[[0,40 ,80],[30,255,255]]
HSV_blue=[[100,90,20],[130,255,170]]

HSV_red=[[150,110,70],[180,255,255],[0,70,120],[20,230,240]]
HSV_green=[[50,100,55],[90,255,200],[0,0,0],[0,0,0]]

# переменные значений HSV цветов 

# ?????????????????????????????????????????????

global_speed=100
# опеределение скорости движение робота

states=['start','main','manual','HSV','finish']
# определение списка стадий программы

porog=0
delta_reg=0
delta_reg_old=0
# переменные необходимые для регулятора двидения

d_red_o,d_green_o=0,0

delta_banka=0
delta_red=0
delta_green=0
area_green=0
area_red=0
red=False
green=False

flag_red=False
flag_green=False

flag_y_red=False
flag_y_green=False
# определение переменных для работы и расчета объезда знаков

flag_min=False
# флаг уменьшения датчика
count_green=False
count_red=False
# флаги счета знаков
boxes=[0,0]
box_map_count=[[0,0],
               [0,0],
               [0,0],
               [0,0]]
box_map=[[0,0,0],[0,0,0],[0,0,0],[0,0,0]]

map_times=[-1,-1,-1,-1]
zona_times=[-1,-1,-1,-1]

procent_map=[-1,-1,-1,-1]
# списки карт времен зон, времен до знаков, знаков
timer_map=0
flag_timer_map=False

timer_zone=-1
flag_zone=False
# таймеры для списков карт
timer_green=0
timer_red=0

timer_count=0
flag_count=False
color_box=0
# таймеры для работы со знаками
c_line=0
count_lines=0
_count_lines=0
timer_line=0
flag_line=False

timer_finish = None
pause_finish = 0.9
# таймеры финиша и знаков
timer_sec=None
secundomer=0
# переменные для екундомера
timer_banka=0
flag_banka=False
delta_banka_old=0
# переменные обозначения знаков
flag_wall_r,flag_wall_l=False,False
# флаги определение стены
direction=None
# переменная определения направления
def black_line_left(hsv,hsv_blue=HSV_blue):

    # больший нижний датчик:

    x1,y1=0,315 # координаты 1й точки датчика
    x2,y2=250,345 # кооржинаты 2й точки датчика

    datb1 = frame[y1:y2,x1:x2]
    # вырезание маленького изображения из камеры
    cv2.rectangle(frame, (x1, y1),(x2, y2), (255, 255, 255), 2)
    # создание контура датчика
    hsv1 = cv2.cvtColor(datb1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    maskd1 = cv2.blur(maskd1,(3,3))

    maskd2 = cv2.inRange(hsv1,np.array(hsv_blue[0]), np.array(hsv_blue[1]))
    maskd2 = cv2.blur(maskd2,(3,3))
    # создание масок черного и синего цветов
    mask=cv2.bitwise_and(cv2.bitwise_not(maskd2),maskd1)
    # вычесление черной маски без синего цвета
    gray1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

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
    # нахождение контура с наибольшей координатой у и обдока его

###################################################################

    # меньший верхний датчик:


    x1,y1=0,280 # координаты 1й точки датчика
    x2,y2=80,315 # кооржинаты 2й точки датчика

    datb1 = frame[y1:y2,x1:x2]
    # вырезание маленького изображения из камеры
    cv2.rectangle(frame, (x1, y1),(x2, y2), (255, 255, 255), 2)
    # создание контура датчика

    hsv1 = cv2.cvtColor(datb1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    maskd1 = cv2.blur(maskd1,(3,3))

    maskd2 = cv2.inRange(hsv1,np.array(hsv_blue[0]), np.array(hsv_blue[1]))
    maskd2 = cv2.blur(maskd2,(3,3))
    # создание масок черного и синего цветов

    mask=cv2.bitwise_and(cv2.bitwise_not(maskd2),maskd1)
    # вычесление черной маски без синего цвета

    gray1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max_s_left1 = 0
    max=0
    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor)
        area = cv2.contourArea(contor)
        if area > 200:
            

            if max < w*h and area>h*w*0.3 and h>10:
                max_s_left1 = (x+w)*h
                max=h*w
                cv2.rectangle(datb1, (x, y), (x + w, y + h), (0, 0, 255), 2)
        cv2.putText(frame, "" + str(max_s_left1+max_s_left), (x1, y1-10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                    (255, 0, 0), 2)
    # нахождение контура с наибольшей координатой у и обдока его

    return max_s_left+max_s_left1
    # вывод суммы площадей контуров обоих левых датчиков

def black_line_right(hsv,hsv_blue=HSV_blue): # функция определения правого  датчика черного бортика

    # больший нижний датчик:

    x1,y1=640-250,315 # координаты 1й точки датчика
    x2,y2=640,345 # кооржинаты 2й точки датчика


    datb1 = frame[y1:y2,x1:x2]
    # вырезание маленького изображения из камеры
    cv2.rectangle(frame, (x1, y1),(x2, y2), (255, 255, 255), 2)
    # создание контура датчика

    hsv1 = cv2.cvtColor(datb1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    maskd1=cv2.blur(maskd1,(3,3))

    maskd2 = cv2.inRange(hsv1,np.array(hsv_blue[0]), np.array(hsv_blue[1]))
    maskd2=cv2.blur(maskd2,(3,3))
    # создание масок черного и синего цветов
    mask=cv2.bitwise_and(cv2.bitwise_not(maskd2),maskd1)
    # вычесление черной маски без синего цвета


    gray1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

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
    # нахождение контура с наибольшей координатой у и обдока его



####################################################

    # меньший верхний датчик:

    x1,y1=640-80,280 # координаты 1й точки датчика
    x2,y2=640,315 # координаты 2й точки датчика

    datb1 = frame[y1:y2,x1:x2]
    # вырезание маленького изображения из камеры
    cv2.rectangle(frame, (x1, y1),(x2, y2), (255, 255, 255), 2)
    # создание контура датчика


    hsv1 = cv2.cvtColor(datb1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    maskd1=cv2.blur(maskd1,(3,3))

    maskd2 = cv2.inRange(hsv1,np.array(hsv_blue[0]), np.array(hsv_blue[1]))
    maskd2=cv2.blur(maskd2,(3,3))
    # создание масок черного и синего цветов
    mask=cv2.bitwise_and(cv2.bitwise_not(maskd2),maskd1)
    # вычесление черной маски без синего цвета

    gray1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    imd1, contours, hod1 = cv2.findContours(mask, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    max_s_right1 = 0
    max=0
    for contor in contours:
        x, y, w, h = cv2.boundingRect(contor)
        area = cv2.contourArea(contor)
        if area >200:

            if max < w*h and area>h*w*0.3 and h>10:
                max=h*w
                max_s_right1 = ((640-x1)-x)*h
                cv2.rectangle(datb1, (x, y), (x + w, y + h), (0, 0, 255), 2)

        cv2.putText(frame, "" + str(max_s_right1+max_s_right), (x1-15,y1-10), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,
                    (255, 0, 0), 2)
    # нахождение контура с наибольшей координатой у и обдока его

    return max_s_right+max_s_right1
    # вывод суммы площадей контуров обоих левых датчиков

def find_start_line(hsv): # функция определения синих и оранжевых линий на повороте
    x1, y1 = 320 - 20, 440 # 1я точка датчика
    x2, y2 = 320 + 20, 480 # 2я точка датчика


    datb1 = frame[y1:y2,x1:x2]
    # вырезание маленького изображения из камеры 
    cv2.rectangle(frame, (x1, y1),(x2, y2), (255, 255, 255), 2)
    # обводка границ датчика
    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    # создание маски цветов
    gray1 = cv2.cvtColor(maskd1, cv2.COLOR_GRAY2BGR)

    imd1, contours, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 80:
            cv2.rectangle(datb1, (x, y), (x+w, y+h), (0, 122, 122), 2)
            return True
    # определение по площади наличие линии
    return False
    # вывод результата
def find_wall(direction,hsv=HSV_black): # функция определения черного бортика перед роботом
    if direction==-1:
        x1, y1 = 180-13, 290
        x2, y2 = 180+3+13, 350  

    if direction==1:
        x1, y1 = 640-180-13, 290 
        x2, y2 = 640-180+13, 350
    # определение 1й и 2й точек в заивсимости от направления

    datb1 = frame[y1:y2,x1:x2]
    # вырезание маленького изображения из камеры
    cv2.rectangle(frame, (x1, y1),(x2, y2), (150, 150, 150), 2)
    # обводка границ датчика
    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    # создание маски цветов
    area_wall = None
    flag_wall=False
    # задача значений флагов
    gray1 = cv2.cvtColor(maskd1, cv2.COLOR_GRAY2BGR)

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
    # определение наличия борика в зависимости от площади контура
    return flag_wall
    # вывод результата вычеслений
def find_box(hsv,color,hsv_o=HSV_orange): # функция определения знаков
    x1, y1 = 30, 230 # координаты 1й точки датчика
    x2, y2 = 640-30, 480-80 # координаты 2й точки датчика

    datb1 = frame[y1:y2,x1:x2]
    # вырезание маленького изображения из камеры
    cv2.rectangle(frame, (x1, y1),(x2, y2), (0, 100, 100), 2)
    # обводка границ датчка
    hsv1 = cv2.cvtColor(datb1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    maskd1=cv2.blur(maskd1,(3,3))
    maskd2 = cv2.inRange(hsv1,np.array(hsv[2]), np.array(hsv[3]))  
    maskd2=cv2.blur(maskd2,(3,3))  
    mask_orange = cv2.inRange(hsv1,np.array(hsv_o[0]), np.array(hsv_o[1]))    
    # создание HSV масок цветов
    mask=cv2.bitwise_and(cv2.bitwise_or(maskd1,maskd2),cv2.bitwise_not(mask_orange))
    # вычесление нуной маски цвета без прочих цветов
    x_banka = 0
    y_banka = 0
    area_banka = 0
    # обнуление значений переменных
    gray1 = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)

    imd1, contours, hod1 = cv2.findContours(maskd1, cv2.RETR_TREE, cv2.CHAIN_APPROX_NONE)
    
    max=0

    x11,y11,w11,h11=0,0,0,0

    for contour in contours:
        x, y, w, h = cv2.boundingRect(contour)
        area = cv2.contourArea(contour)
        if area > 340:
            if y+h>max and (y<(y2-y1-55) or (60<(x+w/2)<(x2-x1-60))):
                max=y+h
                x_banka = x + w/2
                y_banka = y + h
                area_banka = area
                x11,y11,w11,h11=x,y,w,h
    # определение ближайшего знака в зависимости от у координаты и х,у координат контура
    cv2.rectangle(datb1, (x11, y11), (x11+w11, y11+h11), (255,255,255), 2)
    cv2.putText(datb1, str(int(x_banka)) +"-"+ str(int(y_banka)), (x11, y11), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,255,255), 2)            # robot.text_to_frame(frame, area_banka, (x + w, y + h), c, 2) 
    cv2.putText(datb1, color, (x11, y11+15), cv2.FONT_HERSHEY_COMPLEX_SMALL, 1,(255,0,0), 2)            # robot.text_to_frame(frame, area_banka, (x + w, y + h), c, 2) 
    # обводка знаков и вывод значений о контуре
    return [x_banka, y_banka], area_banka
    # возвращение значений координат и площади контура

def count_box(hsv,color,hsv_o=HSV_orange): # функция счета знаков
    x1, y1 = 0, 365 # координата 1й точки датчика
    x2, y2 = 640, 385 # координата 2й точки датчика

    datb1 = frame[y1:y2,x1:x2]
    # вырезание маленького изображения 
    cv2.rectangle(frame, (x1, y1),(x2, y2), (0, 130, 130), 2)
    # обводка границ датчикка
    dat1 = cv2.GaussianBlur(datb1, (5, 5), cv2.BORDER_DEFAULT)
    hsv1 = cv2.cvtColor(dat1, cv2.COLOR_BGR2HSV)
    maskd1 = cv2.inRange(hsv1,np.array(hsv[0]), np.array(hsv[1]))
    maskd1=cv2.blur(maskd1,(3,3))
    maskd2 = cv2.inRange(hsv1,np.array(hsv[2]), np.array(hsv[3]))  
    maskd2=cv2.blur(maskd2,(3,3))  
    mask_orange = cv2.inRange(hsv1,np.array(hsv_o[0]), np.array(hsv_o[1]))    
    # создание HSV масок цветов
    mask=cv2.bitwise_and(cv2.bitwise_or(maskd1,maskd2),cv2.bitwise_not(mask_orange))
    # вычеслене нужной маски с учетом лишних цветов
    x_banka = None
    y_banka = None
    area_banka = None
    # обнуление переменных

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
    # вычесление наличия знака в датчике в зависимости от площади контура
    return flag
    # возвращение результата вычеслений
def telemetry(): # функция вывода телеметрии
    robot.text_to_frame(frame, 'state = ' + str(state), 10, 20,(255,122,122))
    robot.text_to_frame(frame, 'speed = ' + str(global_speed), 10, 40,(0,0,255))
    robot.text_to_frame(frame, 'serv = ' + str(int(p)), 10, 60,(255,255,0))
    robot.text_to_frame(frame, 'fps = ' + str(fps), 505, 20)
    robot.text_to_frame(frame, 'key = ' + str(k), 505, 40,(122,122,255))


    robot.text_to_frame(frame, 'banka = ' + str(int(delta_banka)), 10, 80,(255,255,255))
    robot.text_to_frame(frame,'gr-'+str(flag_green)+' re-'+str(flag_red),10,100,(255,255,255))
    robot.text_to_frame(frame, "FI - Time: " + str(int(secundomer)), 290, 20,(0,0,0)) 

    robot.text_to_frame(frame,str(box_map)+'b_m',10,120,(255,255,255))
    robot.text_to_frame(frame,str(map_times)+'m_t',10,140,(255,255,255))
    robot.text_to_frame(frame,str(zona_times)+'z_t',10,160,(255,255,255))
    robot.text_to_frame(frame,str(procent_map)+'p_m',10,180,(255,255,255))

    if direction is not None:
        if direction==1:
            robot.text_to_frame(frame, 'Lin(+)=' + str(count_lines), 485, 60,(0,0,0))
        else:
            robot.text_to_frame(frame, "Lin(-)=" + str(count_lines), 470, 60,(50,50,50))
    
    robot.text_to_frame(frame, "Bs= " + str(boxes), 470, 80,(255,255,255))
    # вывод необходимых данных на экран 

nup=0
mv=0
lig=0

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
# действи добота сервой при загрузке программы
state='start'
# установление стартовой стадии
time_o=0
timer_line=time.time()
# установление функция нулевого времени
while 1:

    frame = robot.get_frame(wait_new_frame=1)
    # получение картинки с камеры
    fps1 += 1
    if time.time() > fps_time + 1:
        fps_time = time.time()
        fps = fps1
        fps1 = 0
    # счетчик итераций цикла программы в секунду
    k=robot.get_key()
    # получение данных о нажатых на компьютере кнопках
    if k==49:
        state='start'
    if k==50:
        state='main'
        time_o=time.time()

    if k==51:
        state='manual'
    if k==52:
        state='HSV'

    if state=='start': # стадия ожидания запуска основного алгоритма
        robot.light(0,0,0)
        robot.move(0)   
        robot.serv(0)
        # выставление сервомотора по центру при ожидании стартовой кнопки
        if robot.button()==0 or k==50:
            state='main'
            time_o=time.time()
            # запуск стадии самостоятельной езды и перезапись нулевого времени
        else:
            state='start'
    # определение стадий робота в зависимости от нажатых кнопок
    if state=='main': # стадия самостоятельной езды
        if timer_sec==None:
            timer_sec=time.time()
        secundomer=time.time()-timer_sec
        # запсиь таймера секундомера
        if k==187:
            global_speed+=1
        if k==189:
            global_speed-=1
        if k==66:
            robot.tone(200)
        # изменение скорости и звуковой сигнал по нажатию кнопок
        is_orange = find_start_line(HSV_orange)
        is_blue = find_start_line(HSV_blue)
        # опредление наличия синей или оранжевых линий
        if direction is None:
            if is_orange:
                direction = 1
                robot.tone(180)
                count_lines+=1    
            elif is_blue:
                direction = -1
                count_lines+=1
            flag_line = True
            timer_line = time.time()
        # определение направления движения в зависимости от первой найденной линии поворота
        else:
            if count_lines<=12:
                if time.time()>=timer_line+1:
                    if is_orange and direction==-1:
                        flag_line=True
                        timer_line=time.time()
                        count_lines+=1
                    if is_blue and direction==1:
                        flag_line=True
                        timer_line=time.time()
                        count_lines+=1
        # определение линии поворота и увеличение значения счетчика линий
        if count_lines==1 and timer_zone==-1:
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
        # запись времени проезда зон в зависимости от значения счетчика линий
        if int(count_lines) >= 12:
            pause_finish = 0.4*map_times[0]
            if timer_finish is None:
                timer_finish = time.time() + pause_finish
            # запись значения таймера финиша
            if timer_finish is not None and time.time()>=timer_finish:
                robot.tone(255)
                robot.tone(130)
                robot.tone(50)
                print(box_map,'b_m')
                print(map_times,'m_t')
                print(zona_times,'z_t')
                print(procent_map,'p_z')
                print(int(secundomer*10)/10)
                state='finish'
                # действия для обозначения финиша робота и вывод данных
                # изменение стадии робота
            robot.light(255,255,255)


        cord_green,area_green=find_box(HSV_green,'green')
        cord_red,area_red=find_box(HSV_red,'red')
        # определение координат и площади контуров найденных знаков
        delta_banka=0
        
        p=1  # перспектива
        k=-0.2
        d=0.15
        react_area=500
        # определение коэффициентов для расчета отруливания от знаков

        # расчет для зеленого знака
        if area_green is not None and area_green>=react_area:
            if cord_green[1]<110:
                k1=k/2
                d1=d/2
            else:
                k1=k
                d1=d
            # расчет значений коэффициентов в зависимости от у координаты знака
            e=(round(280 - 5 + cord_green[1]*p) - cord_green[0])
            d_green=e*k1+(e-d_green_o)*d1
            d_green_o=e
            delta_banka=d_green
            # расчет ошибки и отрулвания по формуле
            green=True
            if cord_green[1]>130:
                flag_green=True
            flag_red=False
            # изменение значений флагов знаков
            timer_green=time.time()
            # переопределение значения таймера знака
        else:
            if time.time()>=timer_green+0.1:
                green=False
                # переопределение значения флага в зависимости от таймера знака

        # расчет для красного знака
        if area_red is not None and area_red>=react_area:
            if cord_red[1]<110:
                k2=k/2
                d2=d/2
            else:
                k2=k
                d2=d
            # расчет значений коэффициентов в зависимости от у координаты знака

            e=(round(280 + 5 - cord_red[1]*p) - cord_red[0])
            d_red= e*k2+(e-d_red_o)*d2
            d_red_o=e
            delta_banka=d_red
            # расчет ошибки и отрулвания по формуле
            red=True
            if cord_red[1]>130:
                flag_red=True
            flag_green=False
            # изменение значений флагов знаков
            timer_red=time.time()
            # переопределение значения таймера знака
        else:
            if time.time()>=timer_red+0.1:
                red=False
                # переопределение значения флага в зависимости от таймера знака


        if red and green:
            if cord_green[1]>cord_red[1]:
                delta_banka= d_green
                red=False
                flag_red=False
            # перезапись значений определенного знака в зависимости от брольшей у координаты знаков
            else:
                delta_banka= d_red
                green=False
                flag_green=False
            # перезапись значений определенного знака в зависимости от брольшей у координаты знаков

        if red or green:
            timer_banka=time.time()
            delta_banka_old=delta_banka
            flag_min=True
            # перезапись таймера  знака и перезапись значения отъезда и перезапсиь значения флага уменьшения датчика
        else:               
            if time.time()>=timer_banka+0.5:
                flag_min=False
                flag_green,flag_red=False,False
                # перезапись занчения флага уменьшения датчика и флагав знаков


        count_green=count_box(HSV_green,'green')
        count_red=count_box(HSV_red,'red')
        # определение наличия знака в датчике счета знаков
        if flag_count:
            if count_lines>=1 and flag_timer_map==False:
                timer_map=int((time.time()-timer_zone)*10)/10
                # вычисление времени доезда с начала зоны до первого знака
                flag_timer_map=True
                # перезапиь значения флага умменьшения датчика бортика
                if count_lines==1:
                    zona_times[1]=timer_map
                if count_lines==2:
                    zona_times[2]=timer_map
                if count_lines==3:
                    zona_times[3]=timer_map
                if count_lines==4:
                    zona_times[0]=timer_map
                # запись значений времени до первого знака зоны в список в зависимости от значения счетчика линий
        if flag_count and not count_green and not count_red and time.time()>=timer_count+0.2: # определение отсутствия знаков в датчике счета и условие истечение времени по таймеру
            flag_count=False
            boxes[c_box-1]+=1
            # перезапись значения флага для счета знаков  и увеличение значения количесва знаков в списке количества знаков
            if 1<=count_lines<=5:
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
                    if box_map[0]==[0,0,0]:
                        box_map[0][0]=c_box
                    else:
                        box_map[0][2]=c_box
                # запись положения знаков в карту знаков в зависимости первый хнак в зоне или второй

                if 2<=count_lines<=4 and flag_timer_map:
                    if box_map_count[int(count_lines)-1]==[2,0]:
                        box_map[int(count_lines)-1]=[1,0,1]
                    if box_map_count[int(count_lines)-1]==[0,2]:
                        box_map[int(count_lines)-1]=[2,0,2]    
                if count_lines==5 and flag_timer_map:
                    if box_map_count[0]==[2,0]:
                        box_map[0]=[1,0,1]
                    if box_map_count[0]==[0,2]:
                        box_map[0]=[2,0,2]           
                # перезапись значений карты знаков если в зоне 2 одинаковых по цветы знака

                if count_lines==2:
                    timer_map=zona_times[1]
                if count_lines==3:
                    timer_map=zona_times[2]
                if count_lines==4:
                    timer_map=zona_times[3]
                if count_lines==5:
                    timer_map=zona_times[0]
                # определение времени проезда от начала зоны до первого знака в зависимости от значения счетчика линий

                c_l=count_lines-1
                if count_lines==5:
                    c_l=0
                # определение значения переменно для дальнейшей работы с картой знаков в зависимости от значения счетчика линий                    

                prcnt=timer_map/map_times[c_l]
                # вычисление процента проезда до первого знака от всего времени проезда зоны

                if count_lines==2:
                    procent_map[1]=int(prcnt*100)/100
                if count_lines==3:
                    procent_map[2]=int(prcnt*100)/100
                if count_lines==4:
                    procent_map[3]=int(prcnt*100)/100
                if count_lines==5:
                    procent_map[0]=int(prcnt*100)/100
                # запись значений процента доезда до первого знака от всего проезда зоны в карту прозентов зон
                if prcnt<=0.15:
                    if box_map_count[c_l]==[1,0]:
                        box_map[c_l]=[1,0,0]
                    if box_map_count[c_l]==[0,1]:
                        box_map[c_l]=[2,0,0]
                elif prcnt>=0.5:
                    if box_map_count[c_l]==[1,0]:
                        box_map[c_l]=[0,0,1]
                    if box_map_count[c_l]==[0,1]:
                        box_map[c_l]=[0,0,2]
                else:
                    if box_map_count[c_l]==[1,0]:
                        box_map[c_l]=[0,1,0]
                    if box_map_count[c_l]==[0,1]:
                        box_map[c_l]=[0,2,0]
                # вычесление положения знака в зоне в зависимости от процента доезда до первого знака от всего проезда зоны
                if count_lines==4:
                    if box_map_count[0]==[2,0]:
                        box_map[0]=[1,0,1]
                    if box_map_count[0]==[0,2]:
                        box_map[0]=[2,0,2] 

                flag_timer_map=False            
                # перезапись значения флага для дальнейшего опредления новых знаков


        # перезапись значения таймера опредления знака и цвета опредленного знака
        if count_green:
            timer_count=time.time()
            flag_count=True
            c_box=1
        # для зеленого определенного знака
        if count_red:
            timer_count=time.time()
            flag_count=True
            c_box=2
        # для красного определенного знака

        max_l=black_line_left(HSV_black)
        max_r=black_line_right(HSV_black)
        # определение площади найденных контуров черного бортика по датчикам 
        if flag_min and direction is not None:
            if direction==1 and flag_green:
                max_r=0
                robot.light(255,255,255)
            if direction==-1 and  flag_red:
                max_l=0
                robot.light(255,0,255)
        # определение какой датчик нужно уменьшать в зависимости от напрввления и значения флага уменьшения датчика
        delta_reg = max_l - max_r
        # определение разницы между датчиками
        if -50<delta_reg<50:
            delta_reg=0
        # принуление разницы между датчиками если она мала
        delta_reg=delta_reg//50
        # умешьшение значения разницы датчиков
        p = int(delta_reg * 0.15 + (delta_reg - delta_reg_old) * 0.2)
        delta_reg_old = delta_reg
        # вычесление ошибки и перезапись предидущего значения ошибки
        if max_l+max_r==0 and direction is not None:
            p=60*direction
        # определение угла поворота если оба датчика не нашли черный бортик
        else:
            if max_l==0:
                p=-45
            if max_r==0:
                p=45
            # опредление угла поворота если один из датчиков не может определеть черный бортик
        if red or green:
            p=delta_banka
        # переопределение угла поворота если найден знак 

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
            if flag_red ==False and flag_green==False:
                robot.light(0,0,0)
        # заажигание цветов светодиодным модулем в зависимостиот опредленных значений флаго ви переменных
        robot.serv(-p)
        # подача значения угла поворота на сервомотор
        if global_speed<=0:
            global_speed=0
        if global_speed>=255:
            global_speed=255
        # ограничение жначения сокрости вращения мотора
        robot.move(global_speed)   
        # подача значения скорости вращениея мотора на мото
    if state=='finish': # стадия финиша
        robot.light(255,122,122)
        robot.serv(0)
        robot.move(15,False)
        # зажигание светодиода и выставление сервы прямо на стидии финиша
    if state=='manual': # стадия ручного управления
        if k==187:
            global_speed+=1
        if k==189:
            global_speed-=1
        # изменение скорости вращения мотора кнопками на компьютере
        if k==65:
            nup=60
        if k==68:
            nup=-60
        if k==83:
            nup=0
        # изменение угла поворота сервомотора кнопками на компьютере
        if k==87:
            robot.move(global_speed, True)
            time.sleep(0.15)
            # вврадщение мотора вперед
        if k==88:
            robot.move(global_speed, False)
            time.sleep(0.15)
            # ввращение мотора назад
        else:
            robot.move(0, True)
            # остановка вращение мотора
        robot.serv(nup)
        # поворот сермотора на заданный угол
        if k==66:
            robot.tone(200)
        # подача звукового сигнала по кнопке на компьютере
        if robot.button()==0:
            robot.tone(120)
        if k==56:
            robot.light(255,0,0)
        # изменние света светодиода на красный
        if k==57:
            robot.light(0,255,0)
        # изменние света светодиода на зелёный
        if k==48:
            robot.light(0,0,255)
        # изменние света светодиода на синий


    telemetry()
    # вывод телеметрии на экран
    robot.set_frame(frame, 40)
    # получение нового изображения с камеры