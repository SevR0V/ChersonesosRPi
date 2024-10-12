import asyncio
import serial_asyncio
import time
import pigpio
from serialRW import SerialRW
from remote_server import RemoteUdpDataServer

time.sleep(2)

pi = pigpio.pi()
loop = asyncio.get_event_loop()
serialRW = SerialRW()
udp_server = RemoteUdpDataServer(serialRW)


serial_task = serial_asyncio.create_serial_connection(loop, lambda: serialRW, '/dev/serial0', baudrate=9600)
udp_server_task = loop.create_datagram_endpoint(lambda: udp_server, local_addr=('0.0.0.0', 1338))


task = asyncio.gather(udp_server_task, serial_task, return_exceptions=True)
loop.run_until_complete(task)
try:
    loop.run_forever()
except KeyboardInterrupt:
    udp_server.shutdown()
    loop.close()
    pi.stop()
    print("Shutdown")

