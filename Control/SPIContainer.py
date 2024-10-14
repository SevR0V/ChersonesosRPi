import struct
import numpy as np
import time
from enum import IntEnum

# TX_BUFFER: [ MAGIC_START | MOT_SERVO | MAN_Q | ... | MAGIC_END ]
# TX bytes: [MAGIC_START (1x1)(1) | FLAGS (8x1)(8) | LIGHT (2x4)(8) | MOTORS (6x4)(24) | CAM_ANGLE (1x4)(4) | ... | MAGIC_END (1x1)(1) ]
# RX bytes: [MAGIC_START (1x1)(1) | FLAGS (8x1)(8) | EULER (3x4)(12) | EULER_ACC (3x4)(12) | EULER_MAG (3x4)(12) | CUR_ALL (2x4)(4) | CUR_LIGHT1 (1x4)(4) | CUR_LIGHT2 (1x4)(4) | VOLTS24 (1x4)(4) | ... | MAGIC_END (1x1)(1) ]


# class RxBufferOffsets(IntEnum):
#     MAGIC_START = 0
#     FLAGS = 1
#     EULER = 9
#     EULER_ACC = 21
#     EULER_MAG = 33
#     CUR_ALL = 45
#     CUR_LIGHT1 = 49
#     CUR_LIGHT1 = 53
#     VOLTS24 = 57
#     MAGIC_END = 199

# class TxBufferOffsets(IntEnum):
#     MAGIC_START = 0
#     FLAGS = 1
#     LIGHT = 9
#     MOTORS = 17
#     CAM_ANGLE = 41    
#     MAGIC_END = 199

class RxBufferOffsets(IntEnum):
    MAGIC_START = 0
    FLAGS = 1
    MAN_Q = 9
    EULER = 21
    MAGIC_END = 199

class TxBufferOffsets(IntEnum):
    MAGIC_START = 0
    FLAGS = 1
    MOT_SERVO = 9
    MAN_Q = 41
    MAGIC_END = 199

SPI_RX_MAN_Qx_FLAG = lambda x: np.uint64(1 << x)

SPI_RX_EULERx_FLAG = lambda x: np.uint64(1 << x)
SPI_RX_CUR_ALLx_FLAG = lambda x: np.uint64(1 << (1 + x))
SPI_RX_CUR_LIGHTx_FLAG = lambda x: np.uint64(1 << (2 + x))
SPI_RX_VOLTS24x_FLAG = lambda x: np.uint64(1 << (3 + x))


SPI_TX_DES_MOT_SERVOx_FLAG = lambda x: np.uint64(1 << x)
SPI_TX_DES_MAN_Qx_FLAG = lambda x: np.uint64(1 << (1 + x))

SPI_TX_DES_MOTORSx_FLAG = lambda x: np.uint64(1 << x)
SPI_TX_DES_LIGHT_STATEx_FLAG = lambda x: np.uint64(1 << (1 + x))
SPI_TX_DES_CAM_ANGLEx_FLAG = lambda x: np.uint64(1 << (2 + x))


class SPI_Xfer_Container:
    BUFFER_SIZE = 200
    MAGIC_START = 0xAB
    MAGIC_END = 0xCD

    def __init__(self, pi, spi_channel, spi_speed, spi_flags):
        self.pi = pi
        self.spi_handle = self.pi.spi_open(spi_channel, spi_speed, spi_flags)

        self.rx_buffer = bytearray(self.BUFFER_SIZE)
        self.tx_buffer = bytearray(self.BUFFER_SIZE)

        # TX
        self.des_mot_servo = [0.0] * 8
        self.des_man_q = [0.0] * 3

        # RX
        self.man_q = [0.0] * 3
        self.euler = [0.0] * 3
        self.currentAll = 0
        self.cirrentLights = 0
        self.voltage24 = 0

        self.tx_refresh_flags = np.uint64(0)
        self.rx_refresh_flags = np.uint64(0)

    # def _parse_rx_buffer(self):
    #     if (self.rx_buffer[RxBufferOffsets.MAGIC_START] != self.MAGIC_START or
    #         self.rx_buffer[RxBufferOffsets.MAGIC_END] != self.MAGIC_END):
    #         return False

    #     offset = RxBufferOffsets.FLAGS
    #     flags = np.uint64(struct.unpack('q', self.rx_buffer[offset : offset + 8]))
    #     self.rx_refresh_flags = flags

    #     for index in range(0, 3):
    #         if flags & SPI_RX_MAN_Qx_FLAG(index):
    #             offset = RxBufferOffsets.EULER + 4 * index
    #             self.euler[index] = struct.unpack('f', self.rx_buffer[offset : offset + 4])
        
        
    #     return True
    
    def _parse_rx_buffer(self):
        if (self.rx_buffer[RxBufferOffsets.MAGIC_START] != self.MAGIC_START or
            self.rx_buffer[RxBufferOffsets.MAGIC_END] != self.MAGIC_END):
            return False

        offset = RxBufferOffsets.FLAGS
        flags = np.uint64(struct.unpack('q', self.rx_buffer[offset : offset + 8]))
        self.rx_refresh_flags = flags

        for index in range(0, 3):
            if flags & SPI_RX_MAN_Qx_FLAG(index):
                offset = RxBufferOffsets.MAN_Q + 4 * index
                self.man_q[index] = struct.unpack('f', self.rx_buffer[offset : offset + 4])
        
        return True

    def _fill_tx_buffer(self):
        self.tx_buffer[TxBufferOffsets.MAGIC_START] = self.MAGIC_START
        self.tx_buffer[TxBufferOffsets.MAGIC_END] = self.MAGIC_END

        struct.pack_into('q', self.tx_buffer, TxBufferOffsets.FLAGS, self.tx_refresh_flags)

        for index in range(0, 8):
            if self.tx_refresh_flags & SPI_TX_DES_MOT_SERVOx_FLAG(index):
                offset = TxBufferOffsets.MOT_SERVO + 4 * index
                struct.pack_into('f', self.tx_buffer, offset, self.des_mot_servo[index])

        for index in range(0, 3):
            if self.tx_refresh_flags & SPI_TX_DES_MAN_Qx_FLAG(index):
                offset = TxBufferOffsets.MAN_Q + 4 * index
                struct.pack_into('f', self.tx_buffer, offset, self.des_man_q[index])

        self.tx_refresh_flags = np.uint64(0)

    def transfer(self):
        self._fill_tx_buffer()
        count, self.rx_buffer = self.pi.spi_xfer(self.spi_handle, self.tx_buffer)
        return self._parse_rx_buffer()

    # Setters
    def set_mot_servo(self, index, value):
        if 0 <= index < 8:
            self.des_mot_servo[index] = value
            self.tx_refresh_flags |= SPI_TX_DES_MOT_SERVOx_FLAG(index)

    def set_man_q(self, index, value):
        if 0 <= index < 3:
            self.des_man_q[index] = value
            self.tx_refresh_flags |= SPI_TX_DES_MAN_Qx_FLAG(index)

    # Getters
    def get_man_q(self, index):
        if 0 <= index < 3 and self.rx_refresh_flags & SPI_RX_MAN_Qx_FLAG(index):
            self.rx_refresh_flags &= ~(SPI_RX_MAN_Qx_FLAG(index))
            return self.man_q[index]
        return None

    def close(self):
        self.pi.spi_close(self.spi_handle)
        self.pi.stop()

#bridge = SPI_Xfer_Container(0, 500000, 0)

#while True:
#    bridge.set_mot_servo(0, 12.0)
#    bridge.set_mot_servo(4, 100.0)
#    bridge.set_mot_servo(7, -52.0)
#
#    try:
#        # Transfer data over SPI
#        bridge.transfer()
#        # print(", ".join(hex(b) for b in bridge.tx_buffer))
#        print("q1 = ", bridge.get_man_q(0))
#        print("q2 = ", bridge.get_man_q(1))
#        print("q3 = ", bridge.get_man_q(2))
#        # Delay
#        time.sleep(1)
#
#    except:
#        bridge.close()
#        break

