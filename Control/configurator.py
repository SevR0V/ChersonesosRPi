from enum import IntEnum
from control_IMU_types import IMUType
from control_IMU_types import ControlType
import json
import serial
from UARTParser import UART_Xfer_Container
from SPIContainer import SPI_Xfer_Container
from thruster import Thrusters
from yframecontrolsystem import ControlSystem

def read_int_enum(prompt: str, enum_cls: IntEnum) -> IntEnum:
    valid_values = [e.value for e in enum_cls]
    while True:
        s = input(f"{prompt} {valid_values}: ").strip()
        try:
            iv = int(s)
            choice = enum_cls(iv)
            return choice
        except ValueError:
            print(f"Incorrect choise! Please select from this values: {valid_values}.")
            
def ask_yes_no(prompt: str) -> bool:
    while True:
        answer = input(prompt).strip().lower()
        if answer in ('y', 'yes'):
            return True
        if answer in ('n', 'no'):
            return False

try:
    print("Welcome to ROV Configurator.")
    promt = f"""Select control type:
    {ControlType.DIRECT_CTRL}: Direct control from RPi
    {ControlType.STM_SPI_CTRL}: RPi to STM control via SPI
    {ControlType.STM_UART_CTRL}: RPi to STM control via UART
    """
    controlType = read_int_enum(promt, ControlType)
    print(f"Selected control type: {controlType._name_}")

    promt = f"""Select IMU type: 
    {IMUType.STM_IMU}: Embeded on STM IMU
    {IMUType.NAVX}: NavX IMU
    {IMUType.HIWONDER}: HiWonder (witmotion) IMU
    """
    while(True):
        imuType = read_int_enum(promt, IMUType)
        if controlType == ControlType.DIRECT_CTRL:
            if imuType == IMUType.STM_IMU:
                print("!!!!")
                print("!!!!You can't use STM IMU with direct control!!!!")
                print("!!!!")
            else:
                break
    print(f"Selected IMU type: {imuType._name_}")
    
    if not ask_yes_no("Do you want to manage thrusters aligment and direction? (y/n):"):
        exit()
    
    
except Exception as ex:
    print(ex)
except KeyboardInterrupt:
    exit()
