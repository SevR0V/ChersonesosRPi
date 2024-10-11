import time
import numpy as np
from typing import Dict
from utils import map_value, constrain, super_map


class Thrusters:
    def __init__(self, pi, pins, en_pins, direction_mask=None, pulse_zero_deadzones=None, pulse_bounds_deadzones=None, names=None):
        self.pi = pi
        self.thrust_values = [0] * len(pins)
        self.pins = pins
        self.en_pins = en_pins
        self.direction_mask = direction_mask or [1] * len(pins)
        self.pulse_zero_deadzones = pulse_zero_deadzones or [0] * len(pins)
        self.pulse_bounds_deadzones = pulse_bounds_deadzones or [0] * len(pins)
        self.names = names or [f"Motor {i}" for i in range(len(pins))]
        self.min_max_pulse = (1000, 2000)
        self.zero_control_zone = 0.1
        for pin in self.en_pins:
            self.pi.write(pin, 0)

    def initialize(self):
        for pin in self.en_pins:
            self.pi.write(pin,0)
            time.sleep(0.5)
        for pin in self.pins:
            self.pi.set_servo_pulsewidth(pin,1500)
        for pin in self.en_pins:
            self.pi.write(pin,1)
            time.sleep(0.5)
        time.sleep(2)
        print('Thrusters init complete')
#         min_pulse, max_pulse = self.min_max_pulse
#         for pin in self.pins:
#             self.pi.set_servo_pulsewidth(pin, max_pulse)
#         time.sleep(0.2)
#         for pin in self.pins:
#             self.pi.set_servo_pulsewidth(pin, min_pulse)
#         time.sleep(0.2)
#         for pin in self.pins:
#             self.pi.set_servo_pulsewidth(pin, (max_pulse-min_pulse)//2)
#         time.sleep(2)
#         for i in range(1000,2001):
#             for pin in self.pins:
#                 self.pi.set_servo_pulsewidth(pin, i)
#             time.sleep(0.01)
#         for pin in self.pins:
#             self.pi.set_servo_pulsewidth(pin, (max_pulse-min_pulse)//2)




    def get_thrust(self):
        return self.thrust_values

    # @thrust.setter
    # def thrust(self, value):
    #     print value
    #     self._thrust = value
    #
    #     pulse_width = int(map_value(value, -100, 100, 990, 1990))
    #     print pulse_width
    #     self.gpio.set_servo_pulsewidth(self.pin, pulse_width)

    def set_thrust_all(self, values):
        self.thrust_values = [constrain(thrust, -100, 100) * direction for thrust, direction in zip(values, self.direction_mask)]
        min_pulse, max_pulse = self.min_max_pulse
        for pin, thrust, y0_dz, y1_dz in zip(self.pins, self.thrust_values, self.pulse_zero_deadzones, self.pulse_bounds_deadzones):
            #pulse_width = map_value(thrust, -100, 100, min_pulse, max_pulse)
            #if pin == 10:
            #    min_pulse -= 12
            #    max_pulse -= 12
            #else:
            #    min_pulse, max_pulse = self.min_max_pulse    
            pulse_width = super_map(thrust, -100, 100, min_pulse, max_pulse, y0_dz, y1_dz, self.zero_control_zone)
            self.pi.set_servo_pulsewidth(pin, pulse_width)
            #print(pin, pulse_width)

    def set_thrust(self, idx, value):
        pin = self.pins[idx]
        min_pulse, max_pulse = self.min_max_pulse
        #if pin == 10:
         #   min_pulse -= 12
          #  max_pulse -= 12
        y0_dz = self.pulse_zero_deadzones[idx]
        y1_dz = self.pulse_bounds_deadzones[idx]
        thrust = constrain(value, -100, 100) * self.direction_mask[idx]
        pulse_width = map_value(thrust, -100, 100, min_pulse, max_pulse)
        #pulse_width = super_map(thrust, -100, 100, min_pulse, max_pulse, y0_dz, y1_dz, self.zero_control_zone)
        print(pulse_width)
        self.pi.set_servo_pulsewidth(pin, pulse_width)



    def off(self):
        for pin in self.en_pins:
            self.pi.write(pin,0)
        for pin in self.pins:
            self.pi.set_servo_pulsewidth(pin, 0)



