import asyncio
import struct
import math
import time
import ms5837
import numpy as np
from enum import IntEnum
from SPIContainer import SPI_Xfer_Container
from yframecontrolsystem import ControlSystem
from yframecontrolsystem import Axes
from asynctimer import AsyncTimer
from thruster import Thrusters
from navx import Navx
from ligths import Lights
from servo import Servo
from utils import constrain, map_value, ExpMovingAverageFilter
from async_hiwonder_reader import AsyncHiwonderReader
from UARTParser import UART_Xfer_Container
import time
import copy
import serial
import json
from control_IMU_types import ControlType, IMUType

to_rad = math.pi / 180

UDP_FLAGS_MASTERx = np.uint64(1 << 0)
UDP_FLAGS_LIGHT_STATEx = np.uint64(1 << 1)
UDP_FLAGS_STAB_ROLLx = np.uint64(1 << 2)
UDP_FLAGS_STAB_PITCHx = np.uint64(1 << 3)
UDP_FLAGS_STAB_YAWx = np.uint64(1 << 4)
UDP_FLAGS_STAB_DEPTHx = np.uint64(1 << 5)
UDP_FLAGS_RESET_POSITIONx = np.uint64(1 << 6)
UDP_FLAGS_RESET_IMUx = np.uint64(1 << 7)
UDP_FLAGS_UPDATE_PIDx = np.uint64(1 << 8)

YAW_CAP = 0.5



class UDPRxValues(IntEnum):
    FLAGS = 0
    FORWARD = 1
    STRAFE = 2
    VERTICAL = 3
    ROTATION = 4
    ROLL_INC = 5
    PITCH_INC = 6
    POWER_TARGET = 7
    CAM_ROTATE = 8
    MAN_GRIP = 9
    MAN_ROTATE = 10
    ROLL_KP = 11
    ROLL_KI = 12
    ROLL_KD = 13
    PITCH_KP = 14
    PITCH_KI = 15
    PITCH_KD = 16
    YAW_KP = 17
    YAW_KI = 18
    YAW_KD = 19
    DEPTH_KP = 20
    DEPTH_KI = 21
    DEPTH_KD = 22        

class WorkStatus(IntEnum):
    WAITING = -1
    WORKING = 0
    NO_CONNECTION = 1
    TROUBLESHOOTING_DELAY = 2
    EVACUATION = 3
    EVACUATION_ABORT = 4

class RemoteUdpDataServer(asyncio.Protocol):
    def __init__(self, contolSystem: ControlSystem, timer: AsyncTimer, imuType: IMUType, controlType: ControlType, bridge = None, navx: Navx = None, thrusters: Thrusters = None,
                 lights: Lights = None, cameraServo: Servo = None, hiwonderReader: AsyncHiwonderReader = None):
        self.workStatus = WorkStatus.WAITING
        # self.robocorpMPU = robocorpMPU
        self.offlineTimer = 0
        self.evacDepth = 0
        self.controlType = controlType
        self.hiwonderReader = hiwonderReader
        self.imuType = imuType
        self.timer = timer
        self.bridge = None
        if self.controlType == ControlType.STM_SPI_CTRL:
            self.bridge = bridge
        if self.controlType == ControlType.STM_UART_CTRL:
            self.bridge = UART_Xfer_Container()
        self.controlSystem = contolSystem
        self.navx = navx
        self.thrusters = thrusters
        self.lights = lights
        self.cameraServo = cameraServo
        self.remoteAddres = None
        self.timer.subscribe(self.dataCalculationTransfer)
        self.timer.start()
        if self.hiwonderReader is not None:
            self.hiwonderReader.start()
        self.powerTarget = 0
        self.cameraRotate = 0
        self.cameraAngle = 0
        self.lightState = 0
        self.eulers = [0.0, 0.0, 0.0]
        self.accelerations = [0.0, 0.0, 0.0]
        self.gyro = [0.0, 0.0, 0.0]
        self.IMURaw = [0.0, 0.0, 0.0]
        self.eulerMag = [0.0, 0.0, 0.0]
        self.voltage = 0
        self.curAll = 0
        self.curLights = [0.0, 0.0]
        self.depth = 0
        self.MASTER = True
        self.IMUErrors = [0.0, 0.0, 0.0]
        self.incrementScale = 0.5
        self.batCharge = 0
        self.resetIMU = 0
        self.ERRORFLAGS = np.uint64(0)
        self.voltsFilter = ExpMovingAverageFilter(1)
        self.filteredVolts = 0
        
        self.readytoSerialTx = True
        self.serial_retry_timer = 0
        self.serial_safe_timeout_counter = 0
        
        self.blinkTimer = 0
        self.blinkState = False

        self.maxPowerTarget = 1
        
        self.newRxPacket = True
        self.newTxPacket = True
        
        self.depthDelay = 10
        self.counter = 0

        self.time1 = time.time()
        self.time2 = time.time()
        
        if self.controlType == ControlType.STM_UART_CTRL:
                self.robocorpMCU_serial = serial.Serial(
                port='/dev/serial0',  
                baudrate=921600,
                timeout=self.timer.getInterval())
            
        if self.imuType == IMUType.NAVX:
            navx.subscribe(self.navx_data_received)
        
        try:
            self.depth_sensor = ms5837.MS5837(model=ms5837.MODEL_30BA, bus=1)
            self.depth_sensor.init()
        except Exception:
            print('Depth sensor init failed')
            self.ds_init = 0
        else:
            self.ds_init = 1
            print('Depth sensor init complete')        
        time.sleep(2)
        print('Ready to drown!')

    def connection_made(self, transport):        
        self.transport = transport

    def datagram_received(self, data, address):
        packet = data
        if len(packet) == 2 and packet[0] == 0xAA and packet[1] == 0xFF:
            self.remoteAddres = address
            self.workStatus = WorkStatus.WORKING
            print(f"Client {address} connected")
            return
        
        if not self.remoteAddres:
            return
        
        self.offlineTimer = 0
        self.workStatus = WorkStatus.WORKING
        
        if not self.newRxPacket:            
            #fx, fy, vertical_thrust, powertarget, rotation_velocity, manipulator_grip, manipulator_rotate, camera_rotate, reset, light_state, stabilization, RollInc, PitchInc, ResetPosition.
            received = struct.unpack_from("=ffffffffBBBffBffffffffffffB", packet)
            self.powerTarget = received[3] * self.maxPowerTarget
            
            self.controlSystem.setAxisInput(Axes.STRAFE, (received[0] ** 3) * 100 * self.powerTarget)
            self.controlSystem.setAxisInput(Axes.FORWARD, (received[1] ** 3) * 100 * self.powerTarget)
            self.controlSystem.setAxisInput(Axes.DEPTH, (received[2] ** 3) * 100 * self.powerTarget)
            self.controlSystem.setAxisInput(Axes.YAW, (received[4] ** 3) * 100 * self.powerTarget) 

            self.cameraRotate = received[7]
            self.cameraAngle += self.cameraRotate * self.incrementScale * 3
            self.cameraAngle = constrain(self.cameraAngle, 0, 180)
            self.lightState = received[9]

            rollStab =  1 if received[10] & 0b00000001 else 0
            pitchStab = 1 if received[10] & 0b00000010 else 0
            yawStab =   1 if received[10] & 0b00000100 else 0
            depthStab = 1 if received[10] & 0b00001000 else 0

            self.controlSystem.setStabilization(Axes.ROLL, rollStab)
            self.controlSystem.setStabilization(Axes.PITCH, pitchStab)
            self.controlSystem.setStabilization(Axes.YAW, yawStab)
            self.controlSystem.setStabilization(Axes.DEPTH, depthStab)
            
            if received[11]: 
                rollSP = self.controlSystem.getPIDSetpoint(Axes.ROLL) + received[11] * self.incrementScale
                self.controlSystem.setPIDSetpoint(Axes.ROLL, rollSP)
            if received[12]: 
                pitchSP = self.controlSystem.getPIDSetpoint(Axes.PITCH) + received[12] * self.incrementScale
                self.controlSystem.setPIDSetpoint(Axes.PITCH, pitchSP)
            
            if(received[13]):
                self.controlSystem.setPIDSetpoint(Axes.ROLL, 0)
                self.controlSystem.setPIDSetpoint(Axes.PITCH, 0)
                self.controlSystem.setPIDSetpoint(Axes.YAW, self.controlSystem.getAxisValue(Axes.YAW))
                self.controlSystem.setPIDSetpoint(Axes.DEPTH, self.controlSystem.getAxisValue(Axes.DEPTH))
                self.controlSystem.setStabilizations([0,0,0,0,0,0])
                
            if(received[26]):
                self.controlSystem.setPIDConstants(Axes.ROLL, [received[14], received[15], received[16]])
                self.controlSystem.setPIDConstants(Axes.PITCH, [received[17], received[18], received[19]])
                self.controlSystem.setPIDConstants(Axes.YAW, [received[20], received[21], received[22]])
                self.controlSystem.setPIDConstants(Axes.DEPTH, [received[23], received[24], received[25]])
        else:
            # controlFlags, forward, strafe, vertical, rotation, rollInc, pitchInc, powerTarget, cameraRotate, manipulatorGrip, manipulatorRotate, rollKp, rollKi, rollKd, pitchKp, pitchKi, pitchKd, yawKp, yawKi, yawKd, depthKp, depthKi, depthKd
            # flags = MASTER, lightState, stabRoll, stabPitch, stabYaw, stabDepth, resetPosition, resetIMU, updatePID
            received = struct.unpack_from("=Qffffffffffffffffffffff", packet)
            self.MASTER = np.uint64(received[UDPRxValues.FLAGS]) & UDP_FLAGS_MASTERx
            self.lightState = np.uint64(received[UDPRxValues.FLAGS]) & UDP_FLAGS_LIGHT_STATEx
            rollStab =  np.uint64(received[UDPRxValues.FLAGS]) & UDP_FLAGS_STAB_ROLLx
            pitchStab = np.uint64(received[UDPRxValues.FLAGS]) & UDP_FLAGS_STAB_PITCHx
            yawStab =   np.uint64(received[UDPRxValues.FLAGS]) & UDP_FLAGS_STAB_YAWx
            depthStab = np.uint64(received[UDPRxValues.FLAGS]) & UDP_FLAGS_STAB_DEPTHx
            resetPosition = np.uint64(received[UDPRxValues.FLAGS]) & UDP_FLAGS_RESET_POSITIONx
            self.resetIMU = np.uint64(received[UDPRxValues.FLAGS]) & UDP_FLAGS_RESET_IMUx
            updatePID = np.uint64(received[UDPRxValues.FLAGS]) & UDP_FLAGS_UPDATE_PIDx
            
            self.controlSystem.setStabilization(Axes.ROLL, rollStab)
            self.controlSystem.setStabilization(Axes.PITCH, pitchStab)
            self.controlSystem.setStabilization(Axes.YAW, yawStab)
            self.controlSystem.setStabilization(Axes.DEPTH, depthStab) 

            self.powerTarget = received[UDPRxValues.POWER_TARGET] 
            
            self.controlSystem.setAxisInput(Axes.FORWARD, (received[UDPRxValues.FORWARD] ** 3) * 100 * self.powerTarget * self.maxPowerTarget)
            self.controlSystem.setAxisInput(Axes.STRAFE, (received[UDPRxValues.STRAFE] ** 3) * 100 * self.powerTarget * self.maxPowerTarget)
            self.controlSystem.setAxisInput(Axes.DEPTH, (received[UDPRxValues.VERTICAL] ** 3) * 100 * self.powerTarget * self.maxPowerTarget)
            self.controlSystem.setAxisInput(Axes.YAW, (received[UDPRxValues.ROTATION]) * 100 * self.powerTarget * YAW_CAP)

            rollInc = received[UDPRxValues.ROLL_INC]
            pitchInc = received[UDPRxValues.PITCH_INC]

            if rollInc:
                rollSP = self.controlSystem.getPIDSetpoint(Axes.ROLL) + rollInc * self.incrementScale
                self.controlSystem.setPIDSetpoint(Axes.ROLL, rollSP)
            if pitchInc:
                pitchSP = self.controlSystem.getPIDSetpoint(Axes.PITCH) + pitchInc * self.incrementScale
                self.controlSystem.setPIDSetpoint(Axes.PITCH, pitchSP)
                
            self.cameraRotate = received[UDPRxValues.CAM_ROTATE]            
            self.cameraAngle += self.cameraRotate * self.incrementScale * 2
            self.cameraAngle = constrain(self.cameraAngle,-90,90)
            if updatePID:
                self.controlSystem.setPIDConstants(Axes.ROLL, [received[UDPRxValues.ROLL_KP], received[UDPRxValues.ROLL_KI], received[UDPRxValues.ROLL_KD]])
                self.controlSystem.setPIDConstants(Axes.PITCH, [received[UDPRxValues.PITCH_KP], received[UDPRxValues.PITCH_KI], received[UDPRxValues.PITCH_KD]])
                self.controlSystem.setPIDConstants(Axes.YAW, [received[UDPRxValues.YAW_KP], received[UDPRxValues.YAW_KI], received[UDPRxValues.YAW_KD]])
                self.controlSystem.setPIDConstants(Axes.DEPTH, [received[UDPRxValues.DEPTH_KP], received[UDPRxValues.DEPTH_KI], received[UDPRxValues.DEPTH_KD]])
                pids = {
                    "Roll":{
                        "kP": received[UDPRxValues.ROLL_KP],
                        "kI": received[UDPRxValues.ROLL_KI],
                        "kD": received[UDPRxValues.ROLL_KD]
                    },
                    "Pitch":{
                        "kP": received[UDPRxValues.PITCH_KP],
                        "kI": received[UDPRxValues.PITCH_KI],
                        "kD": received[UDPRxValues.PITCH_KD]
                    },
                    "Yaw":{
                        "kP": received[UDPRxValues.YAW_KP],
                        "kI": received[UDPRxValues.YAW_KI],
                        "kD": received[UDPRxValues.YAW_KD]
                    },
                    "Depth":{
                        "kP": received[UDPRxValues.DEPTH_KP],
                        "kI": received[UDPRxValues.DEPTH_KI],
                        "kD": received[UDPRxValues.DEPTH_KD]
                    }
                }
                with open("PIDs.json", "w", encoding="utf-8") as f:
                    json.dump(pids, f, ensure_ascii=False, indent=4)
            
            if resetPosition:
                self.controlSystem.setPIDSetpoint(Axes.ROLL, 0)
                self.controlSystem.setPIDSetpoint(Axes.PITCH, 0)
                self.controlSystem.setPIDSetpoint(Axes.YAW, self.controlSystem.getAxisValue(Axes.YAW))
                self.controlSystem.setPIDSetpoint(Axes.DEPTH, self.controlSystem.getAxisValue(Axes.DEPTH))
                self.controlSystem.setStabilizations([0,0,0,0,0,0])
            
            if self.resetIMU:
                self.IMUErrors = copy.copy(self.eulers)
                       
        self.controlSystem.setdt(self.timer.getInterval())

    def set_PIDs_from_File(self, pids):
        if pids is None:
            return
        self.controlSystem.setPIDConstants(Axes.ROLL, [pids["Roll"]["kP"], pids["Roll"]["kI"], pids["Roll"]["kD"]])
        self.controlSystem.setPIDConstants(Axes.PITCH, [pids["Pitch"]["kP"], pids["Pitch"]["kI"], pids["Pitch"]["kD"]])
        self.controlSystem.setPIDConstants(Axes.YAW, [pids["Yaw"]["kP"], pids["Yaw"]["kI"], pids["Yaw"]["kD"]])
        self.controlSystem.setPIDConstants(Axes.DEPTH, [pids["Depth"]["kP"], pids["Depth"]["kI"], pids["Depth"]["kD"]])

    def navx_data_received(self, sender, data):
        roll, pitch, yaw, heading = data
        self.eulers = [roll, pitch, yaw]
        
    def statusUpdate(self):
        match self.workStatus:
            case WorkStatus.WAITING:
                self.MASTER = False
            case WorkStatus.WORKING:
                if self.offlineTimer >= 1:
                    self.workStatus = WorkStatus.NO_CONNECTION
            case WorkStatus.NO_CONNECTION:
                self.offlineTimer = 0
                self.MASTER = False
                self.workStatus = WorkStatus.TROUBLESHOOTING_DELAY
            case WorkStatus.TROUBLESHOOTING_DELAY:
                if self.offlineTimer >= 5:
                    self.MASTER = True
                    self.workStatus = WorkStatus.EVACUATION
                    self.controlSystem.axesInputs = [0, 0, 0, 0, 0, 0]
                    self.controlSystem.setStabilization(Axes.ROLL, 1)
                    self.controlSystem.setStabilization(Axes.PITCH, 1)
                    self.controlSystem.setStabilization(Axes.DEPTH, 1)
                    self.controlSystem.setPIDSetpoint(Axes.ROLL, 0)
                    self.controlSystem.setPIDSetpoint(Axes.PITCH, 0)
                    self.controlSystem.setPIDSetpoint(Axes.DEPTH, 0.5)                    
                    self.evacDepth = self.controlSystem.getAxisValue(Axes.DEPTH)
                    self.offlineTimer = 0
            case WorkStatus.EVACUATION:
                if self.depth <= 0.5:
                    self.offlineTimer = 0
                    self.MASTER = False
                    return
                else:
                    self.MASTER = True
                if self.offlineTimer >= 10:
                    if abs(self.evacDepth - self.controlSystem.getAxisValue(Axes.DEPTH)) >= 0.3:
                        self.MASTER = False
                        self.workStatus = WorkStatus.EVACUATION_ABORT
                        self.offlineTimer = 0
            case WorkStatus.EVACUATION_ABORT:
                self.MASTER = False

    def dataCalculationTransfer(self):
        self.offlineTimer += self.timer.getInterval()
        self.statusUpdate()
        # print(str(self.workStatus) + str(self.MASTER) + str(self.offlineTimer))
        if self.workStatus > 0:
            self.blinkTimer += self.timer.getInterval()
            if not(self.blinkTimer % 3):
                self.blinkState = not self.blinkState
            if self.blinkState:
                self.lightState = 1
            else:
                self.lightState = 0
        else:
            self.blinkState = False
            self.blinkTimer = 0
        
        # self.time2 = time.time()
        # #print("%.4f"%(self.time2-self.time1))
        # self.time1 = self.time2

        self.counter += 1
        if self.counter >= self.depthDelay:
            self.counter = 0
            if self.ds_init:
                if self.depth_sensor.read(ms5837.OSR_8192):
                    self.depth = self.depth_sensor.pressure(ms5837.UNITS_atm)*10-10

        thrust = self.controlSystem.getThrustersControls()

        # print(*["%.2f" % elem for elem in thrust], sep ='; ')        
        if self.MASTER:
            if self.controlType == ControlType.STM_SPI_CTRL or self.controlType == ControlType.STM_UART_CTRL:
                self.bridge.set_cam_angle_value(self.cameraAngle)
                lightsValues = [50*self.lightState, 50*self.lightState]
                self.bridge.set_lights_values(lightsValues)            
                self.bridge.set_mots_values(thrust)

            if self.controlType == ControlType.DIRECT_CTRL:
                self.thrusters.set_thrust_all(thrust)

                if self.lightState:
                    self.lights.on()
                else:
                    self.lights.off()                
        else:
            thrust = [0.0]*6
            if self.controlType == ControlType.DIRECT_CTRL:
                self.thrusters.set_thrust_all(thrust)
                if self.lightState:
                    self.lights.on()
                else:
                    self.lights.off()
                self.cameraServo.rotate(self.cameraAngle)

            if self.controlType == ControlType.STM_SPI_CTRL or self.controlType == ControlType.STM_UART_CTRL:
                self.bridge.set_cam_angle_value(map_value(round(self.cameraAngle), -90, 90, -100, 100))
                lightsValues = [50*self.lightState, 50*self.lightState]
                self.bridge.set_lights_values(lightsValues)            
                self.bridge.set_mots_values(thrust)

        if self.imuType == IMUType.HIWONDER:
            self.eulers = self.hiwonderReader.getIMUAngles()
        if self.controlType == ControlType.STM_SPI_CTRL or self.controlType == ControlType.STM_UART_CTRL:
            try:
                # Transfer data over SPI
                if self.controlType == ControlType.STM_SPI_CTRL:
                    self.bridge.transfer()
                if self.controlType == ControlType.STM_UART_CTRL:
                    tx_buff = self.bridge.get_TX_buffer()
                    self.robocorpMCU_serial.reset_output_buffer()
                    self.robocorpMCU_serial.write(tx_buff)
                    # print(f"Transmitted: {bytearray(tx_buff)}")
                    if self.robocorpMCU_serial.in_waiting == 150:
                        rx_buffer = self.robocorpMCU_serial.read(self.robocorpMCU_serial.in_waiting)
                        self.bridge.parse_buffer(rx_buffer)
                        self.robocorpMCU_serial.reset_input_buffer()
                        # print(f"Received: {bytearray(rx_buffer)}")
                    else:
                        buf = self.robocorpMCU_serial.read(self.robocorpMCU_serial.in_waiting)
                        # print("No data received, buff size =" + str(self.ser.in_waiting))
                        # print(bytearray(buf))
                        buf_len = len(buf)
                        if buf_len > 0 and buf[buf_len-1] == 0xCD:   
                            # print("Buff offset")                            
                            self.robocorpMCU_serial.reset_input_buffer()
                    
                if self.imuType == IMUType.STM_IMU:   
                    eulers = self.bridge.get_IMU_angles()
                    if eulers is not None:
                        # self.eulers = eulers
                        self.eulers = [-eulers[1],  eulers[0], eulers[2]]
                    acc = self.bridge.get_IMU_accelerometer()
                    if acc is not None:
                        self.accelerations = acc
                    imuRaw = self.bridge.get_IMU_raw()
                    if imuRaw is not None:
                        self.IMURaw = imuRaw
                    mag = self.bridge.get_IMU_magnetometer()
                    if mag is not None:
                        self.eulerMag = mag
                voltage = self.bridge.get_voltage()
                if voltage is not None:
                    self.voltage = voltage
                    self.filteredVolts = self.voltsFilter.update(self.voltage)
                currAll = self.bridge.get_current_all()
                if currAll is not None:
                    self.curAll = currAll
                curLights = self.bridge.get_current_lights()
                if curLights is not None:
                    self.curLights = curLights
                pass
            except Exception as ex:
                print("TRANSFER FAILURE")
                print(ex)

        self.controlSystem.setAxisValue(Axes.DEPTH, self.depth)
        # print(*["%.2f" % elem for elem in self.eulers], sep ='; ')
        # print(*["%.2f" % elem for elem in self.IMUErrors], sep ='; ')
        self.controlSystem.setAxisValue(Axes.ROLL, self.eulers[0] - self.IMUErrors[0])
        self.controlSystem.setAxisValue(Axes.PITCH, self.eulers[1] - self.IMUErrors[1])
        self.controlSystem.setAxisValue(Axes.YAW, self.eulers[2] - self.IMUErrors[2])
        if self.voltage is not None:
            self.batCharge = map_value(self.voltage, 16, 20.5, 0, 100)
        if self.remoteAddres:
            if not self.newTxPacket:
                telemetry_data = struct.pack('=fffffff', self.controlSystem.getAxisValue(Axes.ROLL), 
                                            self.controlSystem.getAxisValue(Axes.PITCH), 
                                            self.controlSystem.getAxisValue(Axes.YAW), 
                                            map_value(self.cameraAngle, 40, 90, -90, 90),
                                            self.controlSystem.getAxisValue(Axes.DEPTH), 
                                            self.controlSystem.getPIDSetpoint(Axes.ROLL), 
                                            self.controlSystem.getPIDSetpoint(Axes.PITCH))
                
                self.transport.sendto(telemetry_data, self.remoteAddres)
            else:
                #ERRORFLAGS, roll, pitch, yaw, depth, batVoltage, batCharge, batCurrent, rollSP, pitchSP
                telemetry_data = struct.pack('=Qfffffffff',
                                            self.ERRORFLAGS,
                                            self.controlSystem.getAxisValue(Axes.ROLL), 
                                            self.controlSystem.getAxisValue(Axes.PITCH), 
                                            self.controlSystem.getAxisValue(Axes.YAW),
                                            self.controlSystem.getAxisValue(Axes.DEPTH),
                                            self.filteredVolts,
                                            self.batCharge,
                                            self.cameraAngle,
                                            self.controlSystem.getPIDSetpoint(Axes.ROLL), 
                                            self.controlSystem.getPIDSetpoint(Axes.PITCH))
                
                self.transport.sendto(telemetry_data, self.remoteAddres)
                

    def shutdown(self):
        self.timer.stop()
        if self.bridge is not None:
            self.bridge.close()
        if self.thrusters is not None:
            self.thrusters.off()
        print("Stop main server")
