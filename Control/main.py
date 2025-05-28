import asyncio
import pigpio
from remoteserver import RemoteUdpDataServer
from control_IMU_types import IMUType
from control_IMU_types import ControlType
from SPIContainer import SPI_Xfer_Container
from asynctimer import AsyncTimer
from yframecontrolsystem import ControlSystem
from yframecontrolsystem import ThrustersNames
from navx import Navx
import serial_asyncio
from thruster import Thrusters
from ligths import Lights
from servo import Servo
from async_hiwonder_reader import AsyncHiwonderReader
import os, json

#select IMU
# IMUType.NAVX
# IMUType.STM_IMU
# IMUType.HIWONDER
imuType = IMUType.STM_IMU

#select contol type
# ControlType.DIRECT_CTRL
# ControlType.STM_SPI_CTRL
controlType = ControlType.STM_UART_CTRL

if controlType == ControlType.DIRECT_CTRL and imuType == IMUType.STM_IMU:
    print("Wrong IMU Type")
    exit()
    imuType = IMUType.NAVX

pi = pigpio.pi()
loop = asyncio.get_event_loop()

bridge = None
navx = None
lights = None
thrusters = None
cameraServo = None
udp_server = None
hiwonderIMU = None
hiwonderTimer = None
robocorp_MPU = None
pids = None

#init thrusters parameters
thrustersOrder = [ThrustersNames.H_REAR, 
                  ThrustersNames.V_FRONT_LEFT,
                  ThrustersNames.V_REAR, 
                  ThrustersNames.V_FRONT_RIGHT,
                  ThrustersNames.H_FRONT_RIGHT,
                  ThrustersNames.H_FRONT_LEFT]
thrustersDirCorr = [1, 1, 1, -1, -1, 1]
trustersXValues = [-100, 100]

#init control system
controlSystem = ControlSystem()
controlSystem.setThrustersCalibrationValues(thrustersDirCorr, thrustersOrder, trustersXValues, 2)

#init timer parameters
timerInterval = 1/300 #300 Hz timer interval

#init timer
timer = AsyncTimer(timerInterval, loop)

if imuType == IMUType.NAVX:
    #init NavX
    navx = Navx()

if imuType == IMUType.HIWONDER:
    hiwonderReader = AsyncHiwonderReader(1/100, loop,'/dev/ttyUSB0', 38400)

if controlType == ControlType.STM_SPI_CTRL:
    #init SPI parameters
    SPIChannel = 0
    SPISpeed = 500000
    SPIFlags = 0
    bridge = SPI_Xfer_Container(pi, SPIChannel, SPISpeed, SPIFlags)

    #init main server
    if imuType == IMUType.NAVX:
        udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, bridge, navx)
    if imuType == IMUType.HIWONDER:
        udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, bridge, hiwonderReader=hiwonderReader)
    if imuType == IMUType.STM_IMU:
        udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, bridge)

if controlType == ControlType.STM_UART_CTRL:
    if imuType == IMUType.NAVX:
        udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, navx=navx)
    if imuType == IMUType.HIWONDER:
        udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, hiwonderReader=hiwonderReader)
    if imuType == IMUType.STM_IMU:
        udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType)

if controlType == ControlType.DIRECT_CTRL:
    #init thrusters
    thrustersPins = [0] * 6

    thrustersPins[thrustersOrder.index(ThrustersNames.H_FRONT_LEFT)]     = 22
    thrustersPins[thrustersOrder.index(ThrustersNames.H_FRONT_RIGHT)]    = 9
    thrustersPins[thrustersOrder.index(ThrustersNames.H_REAR)]           = 27
    thrustersPins[thrustersOrder.index(ThrustersNames.V_FRONT_LEFT)]     = 10
    thrustersPins[thrustersOrder.index(ThrustersNames.V_FRONT_RIGHT)]    = 11
    thrustersPins[thrustersOrder.index(ThrustersNames.V_REAR)]           = 17

    thrusters = Thrusters(pi, thrustersPins, [16], 
                          [[20, 20],[20, 20],[20, 20],[20, 20],[20, 20],[20, 20]],
                          #[[0, 0],[0, 0],[0, 0],[0, 0],[0, 0],[0, 0]],
                          [[0, 0],[0, 0],[0, 0],[0, 0],[0, 0],[0, 0]])

    #init servos
    cameraServo = Servo(pi, 5, [0, 90])

    #init lights
    lights = Lights(pi, [19, 26])

    #init main server
    if imuType == IMUType.NAVX:
        udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, navx=navx, thrusters=thrusters, lights=lights, cameraServo=cameraServo)
    if imuType == IMUType.HIWONDER:
        udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, thrusters=thrusters, 
                                         lights=lights, cameraServo=cameraServo, hiwonderReader=hiwonderReader)

#create async tasks
NavX_serial_task = None
task = None
robocorp_MPU_task = None
udp_server_task = loop.create_datagram_endpoint(lambda: udp_server, local_addr=('0.0.0.0', 1337))
if imuType == IMUType.NAVX:
    NavX_serial_task = serial_asyncio.create_serial_connection(loop, lambda: navx, '/dev/ttyACM0', baudrate=115200)

#register async tasks
if controlType == ControlType.STM_UART_CTRL or controlType == ControlType.STM_SPI_CTRL:
    if imuType == IMUType.NAVX:
        task = asyncio.gather(udp_server_task, NavX_serial_task, return_exceptions=True)
    else:
        task = asyncio.gather(udp_server_task, return_exceptions=True)

# load PIDs values from fle
pids_file_path = "PIDs.json"

if os.path.isfile(pids_file_path):
    with open(pids_file_path, "r", encoding="utf-8") as f:
        pids = json.load(f)
    print("PIDs values loaded")
else:
    pids = {
        "Roll":  {"kP": 5, "kI": 0, "kD": 0},
        "Pitch": {"kP": 5, "kI": 0, "kD": 0},
        "Yaw":   {"kP": 5, "kI": 0, "kD": 0},
        "Depth": {"kP": 200, "kI": 0, "kD": 0},
    }
udp_server.set_PIDs_from_File(pids)

#start main loop
loop.run_until_complete(task)
try:
    loop.run_forever()
except KeyboardInterrupt:
    udp_server.shutdown()
    loop.close()
    pi.stop()
    print("Shutdown")
except Exception as ex:
    print(ex)
