import math
import pigpio


pi = pigpio.pi()

pin_1A = 23
pin_1B = 24
pin_2A = 25
pin_2B = 8
pin_1PWM = 7
pin_2PWM = 12
pin_1EN = 16
pin_2EN = 20

def initialize():
    pi.set_mode(pin_2B, pigpio.OUTPUT)
    pi.set_mode(pin_1A, pigpio.OUTPUT)
    pi.set_mode(pin_1B, pigpio.OUTPUT)
    pi.set_mode(pin_2A, pigpio.OUTPUT)
    #pi.set_mode(pin_1EN, pigpio.OUTPUT) 
    #pi.set_mode(pin_2EN, pigpio.OUTPUT)
    pi.write(pin_2A, 0)
    pi.write(pin_2B, 0)
    #pi.write(pin_2EN, 0)
    pi.write(pin_1A, 0)
    pi.write(pin_1B, 0)
    #pi.write(pin_1EN, 0)
    pi.set_PWM_dutycycle(pin_1PWM, 0)    
    pi.set_PWM_dutycycle(pin_2PWM, 0)


def grip(pwm):
    value = math.fabs(pwm)
    sign = value / pwm if value != 0 else 0
    if sign >= 0:
        pi.write(pin_2A, 1)
        pi.write(pin_2B, 0)
        #pi.write(pin_2EN, 1)
    else:
        pi.write(pin_2A, 0)
        pi.write(pin_2B, 1)
        #pi.write(pin_2EN, 1)
    pi.set_PWM_dutycycle(pin_2PWM, value)


def rotate(pwm):
    value = math.fabs(pwm)
    sign = value / pwm if value != 0 else 0
    if sign >= 0:
        pi.write(pin_1A, 1)
        pi.write(pin_1B, 0)
        #pi.write(pin_1EN, 1)
    else:
        pi.write(pin_1A, 0)
        pi.write(pin_1B, 1)
        #pi.write(pin_1EN, 1)
    #print(sign)
    #print(pwm)
    pi.set_PWM_dutycycle(pin_1PWM, value)


def release():
    pi.set_PWM_dutycycle(pin_1PWM, 0)
    pi.set_PWM_dutycycle(pin_2PWM, 0)
    pi.write(pin_1A, 0)
    pi.write(pin_1B, 0)
    #pi.write(pin_1EN, 0)
    pi.write(pin_2A, 0)
    pi.write(pin_2B, 0)
    #pi.write(pin_2EN, 0)