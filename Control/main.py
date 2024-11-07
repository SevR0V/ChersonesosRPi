import asyncio
import pigpio
from remoteserver import RemoteUdpDataServer
from remoteserver import IMUType
from remoteserver import ControlType
from SPIContainer import SPI_Xfer_Container
from asynctimer import AsyncTimer
from yframecontrolsystem import ControlSystem
from yframecontrolsystem import ThrustersNames
from navx import Navx
import serial_asyncio
from thruster import Thrusters
from ligths import Lights
from servo import Servo
from async_hiwonder_reader import AsyndHiwonderReader

#select IMU
# IMUType.NAVX
# IMUType.POLOLU
# IMUType.HIWONDER
imuType = IMUType.HIWONDER

#select contol type
# ControlType.DIRECT_CTRL
# ControlType.STM_CTRL
controlType = ControlType.DIRECT_CTRL

if controlType == ControlType.DIRECT_CTRL and imuType == IMUType.POLOLU:
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

#init thrusters parameters
thrustersOrder = [ThrustersNames.H_FRONT_LEFT, 
                  ThrustersNames.H_FRONT_RIGHT,
                  ThrustersNames.H_REAR, 
                  ThrustersNames.V_FRONT_LEFT,
                  ThrustersNames.V_FRONT_RIGHT,
                  ThrustersNames.V_REAR]
thrustersDirCorr = [1, -1, 1, 1, -1, 1]
trustersXValues = [-100, 100]

#init control system
controlSystem = ControlSystem()
controlSystem.setThrustersCalibrationValues(thrustersDirCorr, thrustersOrder, trustersXValues, 2)

#init timer parameters
timerInterval = 1/500 #300 Hz timer interval

#init timer
timer = AsyncTimer(timerInterval, loop)

if imuType == IMUType.NAVX:
    #init NavX
    navx = Navx()

if imuType == IMUType.HIWONDER:
    hiwonderReader = AsyndHiwonderReader(1/200, loop,'/dev/ttyUSB0', 57600)

if controlType == ControlType.STM_CTRL:
    #init SPI parameters
    SPIChannel = 0
    SPISpeed = 500000
    SPIFlags = 0
    bridge = SPI_Xfer_Container(pi, SPIChannel, SPISpeed, SPIFlags)

    #init main server
    udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, bridge, navx)

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
        udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, bridge, navx, thrusters, lights, cameraServo)
    if imuType == IMUType.HIWONDER:
        udp_server = RemoteUdpDataServer(controlSystem, timer, imuType, controlType, bridge, navx, thrusters, 
                                         lights, cameraServo, hiwonderReader)

#create tasks
serial_task = None
task = None
udp_server_task = loop.create_datagram_endpoint(lambda: udp_server, local_addr=('0.0.0.0', 1337))
if imuType == IMUType.NAVX:
    serial_task = serial_asyncio.create_serial_connection(loop, lambda: navx, '/dev/ttyACM0', baudrate=115200)

#register tasks
if imuType == IMUType.NAVX:
    task = asyncio.gather(udp_server_task, serial_task, return_exceptions=True)
else:
    task = asyncio.gather(udp_server_task, return_exceptions=True)

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
