from multiprocessing.sharedctypes import Value
import RPi.GPIO as GPIO
import pigpio
import time
import RobotAPI as rapi

GPIO_PWM_SERVO = 13
GPIO_PWM_MOTOR = 12
GPIO_BUZZER=26
GPIO_button=19

GPIO_PIN_CW = 6
GPIO_PIN_CCW = 5

GPIO_RED=16
GPIO_BLUE=20
GPIO_GREEN=21


WORK_TIME = 10
DUTY_CYCLE = 50
FREQUENCY = 100

pi = pigpio.pi()

def arduino_map(x, in_min, in_max, out_min, out_max):
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min

class Motor:
    def __init__(self, pwm_pin: int, pin_CW: int, pin_CCW: int, frequency: int = 100, enable_timer = False, delay: int = 0.1) -> None:
        self.pin_pwm = pwm_pin
        self.freq = frequency
        self.pwm = None
        self.pin_CW = pin_CW
        self.pin_CCW = pin_CCW
        self.started = False
        self.timer_enabled = enable_timer
        self.timer = 0
        self.delay = delay

    def start(self):
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.pin_pwm, GPIO.OUT)
        # GPIO.setup(self.pin_CW, GPIO.OUT)
        # GPIO.setup(self.pin_CCW, GPIO.OUT)

        # self.pwm = GPIO.PWM(self.pin_pwm, self.freq)
        # self.pwm.start(0)
        # GPIO.output(self.pin_CW, GPIO.LOW)
        # GPIO.output(self.pin_CCW, GPIO.LOW)
        
        # pi.set_mode(self.pin_CW, pigpio.OUTPUT)
        # pi.set_mode(self.pin_CCW, pigpio.OUTPUT)
        # pi.set_mode(self.pin_pwm, pigpio.OUTPUT)
        
        pi.set_mode(self.pin_CW, pigpio.OUTPUT)
        pi.set_mode(self.pin_CCW, pigpio.OUTPUT)
        pi.set_mode(self.pin_pwm, pigpio.OUTPUT)
        self.started = True

    def write(self, force: int, clockwise = True):
        assert self.started, "Start servo before using this method."
        if not self.timer_enabled or time.time() > self.timer:
            if clockwise:
                pi.write(self.pin_CCW, 1)
                pi.write(self.pin_CW, 0)
            else:
                pi.write(self.pin_CCW, 0)
                pi.write(self.pin_CW, 1)
            # arduino_map(force, 0, 100, 0, 255)
            pi.set_PWM_dutycycle(self.pin_pwm, force)
            self.timer = time.time() + self.delay

    def stop(self):
        assert self.started, "Start servo before using this method."
        # self.pwm.stop()
        self.started = False

    def cleanup(self):
        GPIO.cleanup()

class Servo:
    def __init__(self, pwm_pin: int, frequency: int = 100) -> None:
        self.pin = pwm_pin
        self.freq = frequency
        self.angle = 0
        self.pwm = None
        self.started = False

    def start(self):
        # GPIO.setmode(GPIO.BCM)
        # GPIO.setup(self.pin, GPIO.OUT)
        # self.pwm = GPIO.PWM(self.pin, self.freq)
        # self.pwm = 
        # self.pwm.start(0)
        self.started = True

    def write(self, angle: int):
        assert self.started, "Start servo before using this method."
        self.angle = angle + 90
        duty = arduino_map(self.angle, 0, 180, 500, 2500)
        # duty = float(self.angle) / 10.0 + 2.5
        # self.pwm.ChangeDutyCycle(duty)
        pi.set_servo_pulsewidth(self.pin, duty)

    def stop(self):
        assert self.started, "Start servo before using this method."
        # self.pwm.stop()
        self.started = False

    def cleanup(self):
        GPIO.cleanup()

class Buzz:
    def __init__(self,GPIO_buzzer):
        self.pin=GPIO_buzzer
        pass

    def start(self):
        pi.set_mode(self.pin,pigpio.OUTPUT)

    def write(self,tone=120):
        pi.set_PWM_dutycycle(self.pin, tone)
        # pi.write(self.pin,1)
        time.sleep(0.2)
        pi.set_PWM_dutycycle(self.pin, 0)


class Button:
    def __init__(self,GPIO_butn):
        self.pin_bt=GPIO_butn

    def start(self):
        # pi.setwarnings(False)
        pi.set_mode(self.pin_bt,pigpio.INPUT)
        pi.set_pull_up_down(self.pin_bt,pigpio.PUD_UP)

    def work(self):
        return pi.read(self.pin_bt)

class Light:
    def __init__(self,GPIO_red,GPIO_green,GPIO_blue):
        self._r=GPIO_red
        self._g=GPIO_green
        self._b=GPIO_blue
        pass

    def start(self):
        pi.set_mode(self._r, pigpio.OUTPUT)
        pi.set_mode(self._g, pigpio.OUTPUT)
        pi.set_mode(self._b, pigpio.OUTPUT)

    def write(self,red,green,blue):
        pi.set_PWM_dutycycle(self._r, red)
        pi.set_PWM_dutycycle(self._g, green)
        pi.set_PWM_dutycycle(self._b, blue)


class GPIORobotApi(rapi.RobotAPI):
    def __init__(self, motor_pwm_pin = GPIO_PWM_MOTOR, motor_cw_pin = GPIO_PIN_CW, motor_ccw_pin = GPIO_PIN_CCW, servo_pin = GPIO_PWM_SERVO, flag_video=True, flag_keyboard=True, flag_pyboard=False, udp_stream=True, udp_turbo_stream=True, udp_event=True):
        super().__init__(flag_video, flag_keyboard, False, flag_pyboard, udp_stream, udp_turbo_stream, udp_event)
        self._servo = Servo(servo_pin)
        self._servo.start()

        self._motor = Motor(motor_pwm_pin, motor_cw_pin, motor_ccw_pin)
        self._motor.start()


        self._buzz = Buzz(GPIO_BUZZER)
        self._buzz.start()

        self._button = Button(GPIO_button)
        self._button.start()

        self._light = Light(GPIO_RED,GPIO_GREEN,GPIO_BLUE)
        self._light.start()

    def move(self, force: int, clockwise = True):
        self._motor.write(force, clockwise)

    def serv(self, angle: int):
        self._servo.write(angle)

    def tone(self,tone: int):
        self._buzz.write(tone)

    def button(self):
        return self._button.work()

    def light(self, red, green, blue):
        self._light.write(red,green,blue)


if __name__ == "__main__":
    m = Motor(GPIO_PWM_MOTOR, GPIO_PIN_CW, GPIO_PIN_CCW)
    m.start()
    m.write(30)
    time.sleep(1)
    m.write(0)
    m.stop()
    m.cleanup()