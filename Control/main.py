import asyncio
import time
import pigpio
from remote_server import RemoteUdpDataServer
from SPIContainer import SPI_Xfer_Container
from asynctimer import AsyncTimer

time.sleep(2)

pi = pigpio.pi()
loop = asyncio.get_event_loop()
#init thrusters parameters
thrustersPins = [10, 9, 17, 22, 27, 11]
thrustersEnPins = [16]
thrustersDirCorr = [-1, -1, 1, 1, 1, -1]
    #trusters deadzones
    # PWM Servo values:
    # 1000...|1000+low bounds DZ|...|1500-low zero DZ|...1500...|1500+high zero DZ|...|2000-high bounds DZ|...2000
thrusterZeroDZ = [[0, 0],[0, 0],[0, 0],[0, 0],[0, 0],[0, 0]]
thrusterBoundsDZ = [[0, 0],[0, 0],[0, 0],[0, 0],[0, 0],[0, 0]]

#init ligths parameters
lightsPins = [19, 26]

#init SPI parameters
SPIChannel = 0
SPISpeed = 500000
SPIFlags = 0 

#init timer parameters
timerInterval = 1/500 #500 Hz timer interval

#init devices
timer = timer = AsyncTimer(timerInterval, loop)
bridge = SPI_Xfer_Container(pi, SPIChannel, SPISpeed, SPIFlags)

#init main server
udp_server = RemoteUdpDataServer(timer, bridge)

#create tasks
udp_server_task = loop.create_datagram_endpoint(lambda: udp_server, local_addr=('0.0.0.0', 1337))

#register tasks
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
