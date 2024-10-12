import asyncio
import struct
import math
import numpy as np
import time
import camera
import manipulator
import ms5837
from utils import constrain, normalized, PID, ExpMovingAverageFilter

to_rad = math.pi / 180


class RemoteUdpDataServer(asyncio.Protocol):
    def __init__(self, timer, bridge):
        self.timer = timer
        self.bridge = bridge
        self.remote_addres = None
        self.thrusters = thrusters
        self.light = light
        timer.subscribe(self.dataCalculationTransfer)
        timer.start()
        self.last_time = 0
        self.camera_angle = 50
        self.heading = 0
        self.angular_velocity = 0
        self.depth = 0
        self.ref_depth = 0
        self.rollStab = 0
        self.pitchStab = 0
        self.yawStab = 0
        self.depthStab = 0
        self.rollPID = PID(10, 0, 0, 0)
        self.pitchPID = PID(10, 0, 0, 0)
        self.depthPID = PID(10, 0, 0, 0)
        self.yawPID = PID(10, 0, 0, 0)
        self.yaw = 0
        self.roll = 0
        self.pitch = 0
        self.depth_filter = ExpMovingAverageFilter(0.8)

#         self.depth_sensor = MS5837
        
        try:
            self.depth_sensor = ms5837.MS5837(model=ms5837.MODEL_30BA, bus=1)
            self.depth_sensor.init()
        except Exception:
            print('Depth sensor init failed')
            self.ds_init = 0
        else:
            self.ds_init = 1
            print('Depth sensor init complete')

        self.reference_thrust_direction = (0, 0)
        self.reference_vertical_thrust = 0
        self.power_target = 0
        self.reference_rotation_velocity = 0
        self.rollSP = 0
        self.pitchSP = 0
        self.rollPI = 0
        self.pitchPI = 0

        self.thrusters.initialize()
#         self.depth_ssensor.init()
        camera.rotate(self.camera_angle)
        time.sleep(2)
        print('Ready to drown!')

    def connection_made(self, transport):
        self.transport = transport

    def calculate_horizontal_thrusters_force(self, direction, angular_velocity):
        fx, fy = direction
        #thrust_force = trunc(np.sqrt(fx*fx + fy*fy), 0, 100)


        #m3 = -fx + angular_velocity
        #m2 = fy * math.sin(120 * to_rad) - fx * math.cos(60 * to_rad) - angular_velocity
        #m1 = fy * math.sin(120 * to_rad) + fx * math.cos(60 * to_rad) + angular_velocity
        
        mdf1 = 0.6 if fx>0 else 1
        mdf2 = 1 if fx>0 else 0.6
        mdf3 = 1.2 if fx>0 else 0.6

        if np.abs(angular_velocity) >= 1 or not self.yawStab : self.yawPID.setSetpoint(self.yaw)
        yaw_pid = self.yawPID.update(self.yaw, self.timer.getInterval()) if (np.abs(angular_velocity) < 1) & self.yawStab else 0
        #print(yaw_pid)

        m1 = fx * mdf1/2 + np.sqrt(3) * fy / 2 + 0.32 * (angular_velocity + yaw_pid)
        m3 = - fx * mdf3 + 0.32 * (angular_velocity + yaw_pid)
        m2 = - fx * mdf2/2 + np.sqrt(3) * fy / 2 - 0.32 * (angular_velocity + yaw_pid)

        
        
        #normalized_horizontal_motors_thrust = normalized([m1, m2, m3])
        #motors_thrust = normalized_horizontal_motors_thrust * thrust_force  

        return m1, m2, m3

    def calculate_vertiacal_thrusters_force(self, roll, pitch, thrust, direction):
        #roll_error = (roll - self.rollSP) #if np.abs(roll) > 1.5 else 0
        #pitch_error = (pitch - self.pitchSP) #if np.abs(pitch) > 1.5 else 0
        _, fy = direction

        #hack disable stabilization
        #roll_error = 0
        #pitch_error = 0
        #kp_roll = -7
        #kp_pitch = -7
        #ki_roll = -0.1
        #ki_pitch = -0.1
        
        if not ((np.abs(thrust)<1) and self.depthStab):
                self.depthPID.setSetpoint(self.depth)
        
        #if np.abs(roll_error) < 5:
        #    self.rollPI += roll_error * ki_roll * self.navx.elapsed_time
        #if np.abs(pitch_error) < 5:
        #    self.pitchPI += pitch_error * ki_pitch * self.navx.elapsed_time
        #roll_PID = roll_error * kp_roll + self.rollPI
        #pitch_PID = pitch_error * kp_pitch + self.pitchPI
        roll_PID = self.rollPID.update(roll, self.timer.getInterval()) if self.rollStab else 0
        pitch_PID = self.pitchPID.update(pitch, self.timer.getInterval()) if self.pitchStab else 0
        depth_PID = -self.depthPID.update(self.depth, self.timer.getInterval()) if self.depthStab else 0
        

        roll_PID = constrain(roll_PID, -100, 100)
        pitch_PID = constrain(pitch_PID, -100, 100)
        depth_PID = constrain(depth_PID, -100, 100)

        #print(f"Roll: {roll_PID:.2f}  Pitch: {pitch_PID:.2f}  Depth: {depth_PID:.2f}")
        motors_thrust = np.array([roll_PID + depth_PID + pitch_PID, -roll_PID + depth_PID + pitch_PID, -pitch_PID + depth_PID]) + thrust
        ###print(motors_thrust)
        #if thrust 
        #normalized_thrust = normalized(motors_thrust)
        #motors_thrust = normalized_thrust * thrust
        return motors_thrust  # [v1, h1, v2, h2, v3, h3]

    def rotate_camera(self, angle, dt):
        self.camera_angle += angle * dt
        self.camera_angle = constrain(self.camera_angle, 0, 90)
        camera.rotate(self.camera_angle)
        ##print("Camera angle:", self.camera_angle)

    def datagram_received(self, data, address):
        ###print("datagram_received")
        packet = data
        #print(packet)
        # ###print(packet)
        if len(packet) == 2 and packet[0] == 0xAA and packet[1] == 0xFF:
            self.remote_addres = address
            #print(f"Client {address} connected")
            return

        if not self.remote_addres:
            return
        ##print(len(packet))
        #fx, fy, vertical_thrust, depth, rotation_velocity, manipulator_grip, manipulator_rotate, camera_rotate, reset, light_state, stabilization, RollInc, PitchInc, ResetPosition.
        #received = struct.unpack_from("=ffffffffB", packet)
        received = struct.unpack_from("=ffffffffBBBffBffffffffffffB", packet)
        ##print(received)

        self.reference_thrust_direction = np.array(received[0:2]) * 255
        self.reference_vertical_thrust = received[2] * 255
        self.power_target = received[3]
        self.reference_rotation_velocity = received[4] * 255

        manipulator_grip = received[5] * 124
        manipulator_rotate = received[6] * 124
        camera_rotate = received[7]
        light_state = received[9]

        self.rollStab =  1 if received[10] & 0b00000001 else 0
        self.pitchStab = 1 if received[10] & 0b00000010 else 0
        self.yawStab =   1 if received[10] & 0b00000100 else 0
        self.depthStab = 1 if received[10] & 0b00001000 else 0
        # self.rollSP += received[11] * self.navx.elapsed_time * 1000
        # self.pitchSP += received[12] * self.navx.elapsed_time * 1000
        
        if received[11]: self.rollPID.setSetpoint(self.rollPID.setpoint + received[11] * self.timer.getInterval() * 1000)
        if received[12]: self.pitchPID.setSetpoint(self.pitchPID.setpoint + received[12] * self.timer.getInterval() * 1000)
        
        if(received[13]):
            self.rollPID.setSetpoint(0)
            self.pitchPID.setSetpoint(0)
            self.yawPID.setSetpoint(self.yaw)
            self.depthPID.setSetpoint(self.depth)
            self.rollStab = 0
            self.pitchStab = 0
            self.yawStab = 0
            self.depthStab = 0
            #print('setToZero')
            
        if(received[26]):
                self.rollPID.setConstants(received[14], received[15], received[16])
                self.pitchPID.setConstants(received[17], received[18], received[19])
                self.yawPID.setConstants(received[20], received[21], received[22])
                self.depthPID.setConstants(received[23], received[24], received[25])
        manipulator.grip(manipulator_grip)
        manipulator.rotate(manipulator_rotate)
        self.rotate_camera(camera_rotate * 1000, self.timer.getInterval())
        # X
        if light_state:
            self.light.on()
            ##print('Lights on')
        else:
            self.light.off()
            ##print('Lights off')
        ##print(light_state)                
   
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
            print("q1 = ", self.bridge.get_man_q(0))
            print("q2 = ", self.bridge.get_man_q(1))
            print("q3 = ", self.bridge.get_man_q(2))
        except:
            self.bridge.close() 
        
        horizontal_motors_thrust = self.calculate_horizontal_thrusters_force(self.reference_thrust_direction, self.reference_rotation_velocity)
        vertical_motors_thrust = self.calculate_vertiacal_thrusters_force(self.roll, self.pitch, self.reference_vertical_thrust, self.reference_thrust_direction)

        h1, h2, h3 = horizontal_motors_thrust 
        v1, v2, v3 = vertical_motors_thrust     

        if self.ds_init:
            if self.depth_sensor.read(ms5837.OSR_256):
                depth = self.depth_sensor.pressure(ms5837.UNITS_atm)*10-10
        else:
            depth = 0        

        self.depth = self.depth_filter.update(depth)
       
        if self.remote_addres:
                #telemetry_data = struct.pack('=ffff', roll, pitch, yaw, heading)
                telemetry_data = struct.pack('=fffffff', self.roll, self.pitch, self.yaw, 0, self.depth, self.rollPID.setpoint, self.pitchPID.setpoint)
                self.transport.sendto(telemetry_data, self.remote_addres)
        

            

    def shutdown(self):
        self.thrusters.off()
        self.light.off()

