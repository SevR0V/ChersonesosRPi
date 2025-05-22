import asyncio
import re
import time
from UARTParser import UART_Xfer_Container


class RobocorpMPU(asyncio.Protocol):
    def __init__(self):
        self.BUFFER_SIZE = UART_Xfer_Container.BUFFER_SIZE
        self.rx_buffer = bytearray(self.BUFFER_SIZE)
        self.tx_buffer = bytearray(self.BUFFER_SIZE)
        self.transport = None
        self.on_data = None
        self.elapsed_time = 0
        self.last_time = time.process_time()

    def subscribe(self, callback):
        self.on_data = callback

    def connection_made(self, transport):
        transport.serial.rts = False
        self.transport = transport
        # command = UART_Xfer_Container.MAGIC_START
        # transport.write(command)

    def data_received(self, data):        
        # print(data)
        if not len(data) == self.BUFFER_SIZE:
            return
        self.rx_buffer = data
        self.on_data(self, self.rx_buffer)
    
    def send_data(self, data):
        self.transport.write(data)


    def connection_lost(self, exc):
        asyncio.get_event_loop().stop()
        
