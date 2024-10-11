import pigpio
from utils import map_value, constrain

pi = pigpio.pi()
camera_pin = 5 

def rotate(angle):
    angle = constrain(angle, 0, 90)
    pulse_width = map_value(angle, 0, 90, 1000, 2000)
    pi.set_servo_pulsewidth(camera_pin, pulse_width)
