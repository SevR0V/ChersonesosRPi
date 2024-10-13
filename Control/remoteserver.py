import asyncio
import struct
import math
import time
import ms5837
from yframecontrolsystem import YFrameControlSystem

to_rad = math.pi / 180

class RemoteUdpDataServer(asyncio.Protocol):
    def __init__(self, timer, bridge):
        self.timer = timer
        self.bridge = bridge
        self.remoteAddres = None
        timer.subscribe(self.dataCalculationTransfer)
        timer.start()
        self.controlSystem = YFrameControlSystem()
        self.powerTarget = 0
        self.cameraRotate = 0
        self.lightState = 0
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
    '''
    def navx_data_received(self, sender, data):
        pitch, roll, yaw, heading = data
        roll += 0  # HACK: error compensation
        pitch += 0
        roll *= -1
        

        self.angular_velocity = sender.angular_velocity

        ####print(f"RPY {roll:.2f} {pitch:.2f} {yaw:.2f} Angular velocity {self.angular_velocity:.2f}")

        horizontal_motors_thrust = self.calculate_horizontal_thrusters_force(self.reference_thrust_direction, self.reference_rotation_velocity)
        vertical_motors_thrust = self.calculate_vertiacal_thrusters_force(roll, pitch, self.reference_vertical_thrust, self.reference_thrust_direction)

        h1, h2, h3 = horizontal_motors_thrust 
        v1, v2, v3 = vertical_motors_thrust

        ####print("Motors before:", v1, h1, v2, h2, v3, h3)
        self.thrusters.set_thrust_all(np.multiply([v1, h1, v2, h2, v3, h3], self.power_target))
        ####print(" Motors after:", np.multiply([v1, h1, v2, h2, v3, h3], self.camera_angle))
        ##print(self.power_target)
        if self.ds_init:
            if self.depth_sensor.read(ms5837.OSR_256):
                depth = self.depth_sensor.pressure(ms5837.UNITS_atm)*10-10
        else:
            depth = 0
        ####print("Depth:", depth)
        self.depth = self.depth_filter.update(depth)
        self.yaw = yaw
        #print(f"Signal: {depth:.2f} Filtered: {self.cur_depth:.2f}")
        if self.remote_addres:
            #telemetry_data = struct.pack('=ffff', roll, pitch, yaw, heading)
            telemetry_data = struct.pack('=fffffff', roll, pitch, yaw, heading, self.depth, self.rollPID.setpoint, self.pitchPID.setpoint)
            self.transport.sendto(telemetry_data, self.remote_addres)
    '''

    def dataCalculationTransfer(self):
        self.bridge.set_mot_servo(0, 12.0)
        self.bridge.set_mot_servo(4, 100.0)
        self.bridge.set_mot_servo(7, -52.0)
        try:
            # Transfer data over SPI
            self.bridge.transfer()
            # print(", ".join(hex(b) for b in bridge.tx_buffer))
            #print("q1 = ", self.bridge.get_man_q(0))
            #print("q2 = ", self.bridge.get_man_q(1))
            #print("q3 = ", self.bridge.get_man_q(2))
        except:
            print("SPI TRANSFER FAILURE")

        if self.ds_init:
            if self.depth_sensor.read(ms5837.OSR_256):
                depth = self.depth_sensor.pressure(ms5837.UNITS_atm)*10-10
        else:
            depth = 0        

        hfl, hfr, hr, vfl, vfr, vr   = tuple(self.controlSystem.getMotsControls())
        print("HFL: ", hfl, "HFR: ", hfr, "HR: ", hr, "VFL: ", vfl, "VFR: ", vfr, "VR:", vr)

        if self.remoteAddres:
            telemetry_data = struct.pack('=fffffff', vfl, vfr, vr, 0, depth, self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.ROLL), self.controlSystem.getPIDSetpoint(self.controlSystem.ControlAxes.PITCH))
            self.transport.sendto(telemetry_data, self.remoteAddres)
        

            

    def shutdown(self):       
        self.bridge.close()  
        print("Stop main server")
