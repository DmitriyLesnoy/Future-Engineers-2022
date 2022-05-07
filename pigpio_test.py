from multiprocessing.sharedctypes import Value
import pigpio
import time
import RobotAPI as rapi

GPIO_PWM_SERVO = 13
GPIO_PWM_MOTOR = 12 #12
GPIO_BUZZER=19
GPIO_button=6

GPIO_PIN_CW = 6 #27
GPIO_PIN_CCW = 5 #17
 
WORK_TIME = 10
DUTY_CYCLE = 50
FREQUENCY = 100

pi = pigpio.pi()

pi.set_mode(GPIO_PIN_CCW, pigpio.OUTPUT)
pi.set_mode(GPIO_PIN_CW, pigpio.OUTPUT)
pi.set_mode(GPIO_PWM_MOTOR, pigpio.OUTPUT)


while 1:

    # pi.write(GPIO_PWM_MOTOR, 1)
    pi.write(GPIO_PIN_CCW, 1)
    pi.write(GPIO_PIN_CW, 0)
    # pi.write(12, 1)
    # arduino_map(force, 0, 100, 0, 255)
    pi.set_PWM_dutycycle(GPIO_PWM_MOTOR,70)
    print(pi.read(GPIO_PWM_MOTOR), pi.read(GPIO_PIN_CCW),pi.read(GPIO_PIN_CW),pi.read(12))
    time.sleep(1)
