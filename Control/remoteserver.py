import asyncio
import struct
import math
import time
import ms5837
from SPIContainer import SPI_Xfer_Container
from yframecontrolsystem import YFrameControlSystem

to_rad = math.pi / 180

class RemoteUdpDataServer(asyncio.Protocol):
    def __init__(self, contolSystem: YFrameControlSystem, timer, bridge: SPI_Xfer_Container, thrustersDirCorr):
        self.timer = timer
        self.bridge = bridge
        self.remoteAddres = None
        timer.subscribe(self.dataCalculationTransfer)
        timer.start()
        self.controlSystem = contolSystem
        self.powerTarget = 0
        self.cameraRotate = 0
        self.cameraAngle = 0
        self.lightState = 0
        self.eulers = [0.0, 0.0, 0.0]
        self.accelerations = [0.0, 0.0, 0.0]
        self.eulerMag = [0.0, 0.0, 0.0]
        self.voltage = 0
        self.curAll = 0
        self.curLights = [0.0, 0.0]
        self.depth = 0
        
        self.thrustersDirCorr = thrustersDirCorr
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
            print(f"Client {address} connected")
            return
        
        if not self.remoteAddres:
            return
        
        #fx, fy, vertical_thrust, powertarget, rotation_velocity, manipulator_grip, manipulator_rotate, camera_rotate, reset, light_state, stabilization, RollInc, PitchInc, ResetPosition.
        received = struct.unpack_from("=ffffffffBBBffBffffffffffffB", packet)

        self.controlSystem.setAxisInput(self.controlSystem.ControlAxes.STRAFE, received[0] * 100)
        self.controlSystem.setAxisInput(self.controlSystem.ControlAxes.FORWARD, received[1] * 100)
        self.controlSystem.setAxisInput(self.controlSystem.ControlAxes.DEPTH, received[2] * 100)
        self.controlSystem.setAxisInput(self.controlSystem.ControlAxes.YAW, received[4] * 100) 

        self.powerTarget = received[3]
        self.cameraRotate = received[7]
        self.cameraAngle += self.cameraRotate * self.timer.getInterval() * 250
        self.lightState = received[9]

        rollStab =  1 if received[10] & 0b00000001 else 0
        pitchStab = 1 if received[10] & 0b00000010 else 0
        yawStab =   1 if received[10] & 0b00000100 else 0
        depthStab = 1 if received[10] & 0b00001000 else 0

        self.controlSystem.setStabilization(self.controlSystem.ControlAxes.ROLL, rollStab)
        self.controlSystem.setStabilization(self.controlSystem.ControlAxes.PITCH, pitchStab)
        self.controlSystem.setStabilization(self.controlSystem.ControlAxes.YAW, yawStab)
        self.controlSystem.setStabilization(self.controlSystem.ControlAxes.DEPTH, depthStab)
        
        if received[11]: 
            rollSP = self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.ROLL) + received[11] * self.timer.getInterval() * 250
            self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.ROLL, rollSP)
        if received[12]: 
            pitchSP = self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.PITCH) + received[12] * self.timer.getInterval() * 250
            self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.PITCH, pitchSP)
        
        if(received[13]):
            self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.ROLL, 0)
            self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.PITCH, 0)
            self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.YAW, self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.YAW))
            self.controlSystem.setPIDSetpoint(self.controlSystem.ControlAxes.DEPTH, self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.DEPTH))
            self.controlSystem.setStabilizations([0,0,0,0,0,0])
            
        if(received[26]):
                self.controlSystem.setPIDConstants(self.controlSystem.ControlAxes.ROLL, [received[14], received[15], received[16]])
                self.controlSystem.setPIDConstants(self.controlSystem.ControlAxes.PITCH, [received[17], received[18], received[19]])
                self.controlSystem.setPIDConstants(self.controlSystem.ControlAxes.YAW, [received[20], received[21], received[22]])
                self.controlSystem.setPIDConstants(self.controlSystem.ControlAxes.DEPTH, [received[23], received[24], received[25]])           
        self.controlSystem.setdt(self.timer.getInterval())
        self.controlSystem.updateControl()

    def dataCalculationTransfer(self):        
        self.controlSystem.updateControl()
        if self.ds_init:
            if self.depth_sensor.read(ms5837.OSR_256):
                self.depth = self.depth_sensor.pressure(ms5837.UNITS_atm)*10-10
            
        self.bridge.set_cam_angle_value(self.cameraAngle)
        lightsValues = [50*self.lightState, 50*self.lightState]
        self.bridge.set_lights_values(lightsValues)
        self.bridge.set_mots_values(self.controlSystem.getMotsControls())
        self.bridge.set_cam_angle_value(self.cameraAngle)     
        try:
            # Transfer data over SPI
            self.bridge.transfer()
        except:
            print("SPI TRANSFER FAILURE")        
        eulers = self.bridge.get_IMU_angles()
        if eulers is not None:
            self.eulers = eulers 
        else:
            print("Gyro data read error")           
        voltage = self.bridge.get_voltage()
        if voltage is not None:
            self.voltage = voltage
        else:
            print("Voltage read error") 
        currAll = self.bridge.get_current_all()
        if currAll is not None:
            self.curAll = currAll
        else:
            print("Current read error") 
        curLights = self.bridge.get_current_lights()
        if curLights is not None:
            self.curLights = curLights
        else:
            print("Lights current read error")
        acc = self.bridge.get_IMU_accelerometer()
        if acc is not None:
            self.accelerations = acc
        else:
            print("Accelerations read error")
        mag = self.bridge.get_IMU_magnetometer()
        if mag is not None:
            self.eulerMag = mag
        else:
            print("Magnetometer date read error")
        self.controlSystem.setAxesValues([0, 0, self.depth, self.eulers[0], self.eulers[1], self.eulers[2]])

        if self.remoteAddres:
            telemetry_data = struct.pack('=fffffff', self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.ROLL), 
                                         self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.PITCH), 
                                         self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.YAW), 
                                         0.0, 
                                         self.controlSystem.getAxisValue(self.controlSystem.ControlAxes.DEPTH), 
                                         self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.ROLL), 
                                         self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.PITCH))
            self.transport.sendto(telemetry_data, self.remoteAddres)

    def shutdown(self):       
        self.bridge.close()  
        print("Stop main server")
