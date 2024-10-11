import serial
import time

if __name__ == '__main__':
    connection = serial.Serial('/dev/ttyACM0', 115200)
    for crc in range(0xFF):
        command = b"!Sy32%X\r\n" % crc
        # print("Send [%s]" % command)
        connection.write(command)
        responce = connection.read_all()
        try:
            if responce.index(b"!sy") >= 0:
                print("!!! %s !!!" % command)
                print(responce)
        except:
            pass
        # print("Responce [%s]" % responce)
        time.sleep(0.1)
