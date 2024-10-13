import asyncio

class SerialRW(asyncio.Protocol):
	def __init__(self):
		self.transport = None
		self.on_data = None
		self.data = ''
		
	def connection_made(self, transport):
		self.transport = transport
		print('port opened', transport)
		transport.serial.rts = False  # You can manipulate Serial object via transport
		#transport.write(b'Hello, World!\n')  # Write serial data via transport

	def data_received(self, data):
		self.data = self.data + str(data)[2:-1]
		if b'\n' in data:
			print(f"Recieved from UART: {self.data}")
			self.on_data(self,self.data[0:-4])
			self.data = ''		
			
	def send_data(self, data):
		print(f"Sending to UART: {data}")
		self.transport.write(data+b'\n')

	def connection_lost(self, exc):
		print('port closed')
		self.transport.loop.stop()

	def pause_writing(self):
		print('pause writing')
		print(self.transport.get_write_buffer_size())

	def resume_writing(self):
		print(self.transport.get_write_buffer_size())
		print('resume writing')

	def subscribe(self, callback):
		self.on_data = callback
