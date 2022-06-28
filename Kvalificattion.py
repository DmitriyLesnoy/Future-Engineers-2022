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


flag_min=False
# флаг уменьшения датчика
count_green=False
count_red=False
# флаги счета знаков

timer_map=0
flag_timer_map=False

timer_zone=-1
flag_zone=False
# таймеры для списков карт
timer_green=0
timer_red=0

timer_count=0
flag_count=False
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

        if int(count_lines) >= 12:
            pause_finish = 0.4*map_times[0]
            if timer_finish is None:
                timer_finish = time.time() + pause_finish
            # запись значения таймера финиша
            if timer_finish is not None and time.time()>=timer_finish:
                robot.tone(255)
                robot.tone(130)
                robot.tone(50)
                print(int(secundomer*10)/10)
                state='finish'
                # действия для обозначения финиша робота и вывод данных
                # изменение стадии робота
            robot.light(255,255,255)


        

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


          
        if flag_line:
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