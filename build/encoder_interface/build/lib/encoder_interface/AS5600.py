import sys
import smbus
import time

from encoder_interface.AS5600Constants import AS5600Constants as A

class AS5600:
    __dev_id = 0

    def __init__(self, a_bus=1, a_address=0x36):

        self.__dev_id = a_address
        self.__bus = smbus.SMBus(a_bus)
        pass

    def getPosition(self):
        ang_msb = self.__bus.read_byte_data(self.__dev_id,A._RAWANGLEAddressMSB)
        ang_lsb = self.__bus.read_byte_data(self.__dev_id,A._RAWANGLEAddressLSB)
        ang = (ang_msb<<8)|(ang_lsb)
        return ang