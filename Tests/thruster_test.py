import pigpio
import time
from utils import constrain 
from thruster import Thrusters

pi = pigpio.pi()
#mot_pins = [10, 22, 11, 9, 17, 27] #[5 2 3 6 1 4] [11, 9, 10, 27, 22, 17]
mot_pins = [17]
#pi.write(16,0)
time.sleep(1)
for pin in mot_pins:
    pi.set_servo_pulsewidth(pin, 1500)
#pi.write(16,1)
time.sleep(3)

try:
    while 1:
        pulse_width = input('Set signal value: ')
        pulse_width = int(pulse_width) if pulse_width else 0
        pulse_width = constrain(pulse_width, 500, 2500) if pulse_width else 1500
        print('Setting', pulse_width, 'to motors!')
        for pin in mot_pins:
            pi.set_servo_pulsewidth(pin, pulse_width)
except KeyboardInterrupt:
    #pi.write(16,0)
    for pin in mot_pins:
        pi.set_servo_pulsewidth(pin, 1500)
    pi.stop()
    print('beep')

    
