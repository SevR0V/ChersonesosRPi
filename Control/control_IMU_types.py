from enum import IntEnum

class IMUType(IntEnum):
    STM_IMU = 0
    NAVX = 1
    HIWONDER = 2

class ControlType(IntEnum):
    DIRECT_CTRL = 0
    STM_SPI_CTRL = 1
    STM_UART_CTRL = 2