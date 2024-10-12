import asyncio
class RemoteUdpDataServer(asyncio.Protocol):
        def __init__(self, serialRW):
                
                self.remote_addres = None
                print('Ready to send perdezh')
                
                self.serialRW = serialRW
                serialRW.subscribe(self.serialRW_data_received)
                
        def connection_made(self, transport):
                self.transport = transport

   
        def datagram_received(self, data, address):
                packet = data
                if len(packet) == 2 and packet[0] == 0xAA and packet[1] == 0xFF:
                        self.remote_addres = address
                        print(f"Client {address} connected")                        
                        return
                print(f"Recieved from udp: {packet}")

                if not self.remote_addres:
                        return
                #received = struct.unpack_from("=ffffffffBBBffBffffffffffffB", packet)
                self.serialRW.send_data(packet)
        
        def serialRW_data_received(self, sender, data):
                print(f"Sending to udp: {data}")
                if self.remote_addres:
                        #telemetry_data = struct.pack('=fffffff', data)
                        self.transport.sendto(bytes(data,'utf-8'), self.remote_addres)
            


        def shutdown(self):
                print('Perdezh is ended')

