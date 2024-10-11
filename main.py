import asyncio
import serial_asyncio
import time
import pigpio
from navx import Navx
from remote_server import RemoteUdpDataServer
from thruster import Thrusters
from os import system
from ligths import Lights

time.sleep(2)


pi = pigpio.pi()
loop = asyncio.get_event_loop()
thrusters = Thrusters(pi, [10, 9, 17, 22, 27, 11], [16], [-1, -1, 1, 1, 1, -1], [[0, 0],[0, 0],[0, 0],[0, 0],[0, 0],[0, 0]], [[0, 0],[0, 0],[0, 0],[0, 0],[0, 0],[0, 0]])
light = Lights(pi, [19, 26])
navx = Navx()
udp_server = RemoteUdpDataServer(navx, thrusters, light)


udp_server_task = loop.create_datagram_endpoint(lambda: udp_server, local_addr=('0.0.0.0', 1337))
serial_task = serial_asyncio.create_serial_connection(loop, lambda: navx, '/dev/ttyACM0', baudrate=115200)


task = asyncio.gather(udp_server_task, serial_task, return_exceptions=True)
loop.run_until_complete(task)
try:
    loop.run_forever()
except KeyboardInterrupt:
    udp_server.shutdown()
    loop.close()
    pi.stop()
    print("Shutdown")

