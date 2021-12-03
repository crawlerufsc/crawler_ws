import smbus2 as smbus
import ctypes


class I2cdev:
  def __init__(self, a_bus=1):
    self.bus = smbus.SMBus(a_bus)

  # I2Cdev::I2Cdev() { }

  # void I2Cdev::initialize() {
  #   bcm2835_init();
  #   bcm2835_i2c_set_baudrate( i2c_baudrate  );
  # }

  # /** Enable or disable I2C, 
  #  * @param isEnabled true = enable, false = disable
  #  */
  # void I2Cdev::enable(bool isEnabled) {
  #   if ( set_I2C_pins ){
  #     if (isEnabled)
  #       bcm2835_i2c_end();
  #     else
  #       bcm2835_i2c_begin() ;
  #   }
  # }




  # /** Read a single bit from an 8-bit device register.
  #  * @param devAddr I2C slave device address
  #  * @param regAddr Register regAddr to read from
  #  * @param bitNum Bit position to read (0-7)
  #  * @param data Container for single bit value
  #  * @return Status of read operation (true = success)
  #  */
  # int8_t - uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t *data
  def readBit(self, devAddr, regAddr, bitNum):
    data = self.bus.read_byte_data(devAddr, regAddr)
    return data & (1 << bitNum)
    

  # /** Read multiple bits from an 8-bit device register.
  #  * @param devAddr I2C slave device address
  #  * @param regAddr Register regAddr to read from
  #  * @param bitStart First bit position to read (0-7)
  #  * @param length Number of bits to read (not more than 8)
  #  * @param data Container for right-aligned value (i.e. '101' read from any bitStart position will equal 0x05)
  #  * @return Status of read operation (true = success)
  #  */
  # int8_t
  # def readBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t *data):
  #   # // 01101001 read byte
  #   # // 76543210 bit numbers
  #   # //    xxx   args: bitStart=4, length=3
  #   # //    010   masked
  #   # //   -> 010 shifted
  #   bcm2835_i2c_setSlaveAddress(devAddr);
  #   sendBuf[0] = regAddr;
  #   uint8_t response = bcm2835_i2c_write_read_rs(sendBuf, 1, recvBuf, 1);
  #   uint8_t b = (uint8_t) recvBuf[0];
  #   if (response == BCM2835_I2C_REASON_OK) {
  #     uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
  #     b &= mask;
  #     b >>= (bitStart - length + 1);
  #     *data = b;
  #   }
  #   return response == BCM2835_I2C_REASON_OK;
  
  def readBits(self, devAddr, a_reg_add, a_bit_start, a_length):
    byte = self.bus.read_byte_data(devAddr, a_reg_add)
    mask = ((1 << a_length) - 1) << (a_bit_start - a_length + 1)
    byte &= mask
    byte >>= a_bit_start - a_length + 1
    return byte

  # /** Read single byte from an 8-bit device register.
  #  * @param devAddr I2C slave device address
  #  * @param regAddr Register regAddr to read from
  #  * @param data Container for byte value read from device
  #  * @return Status of read operation (true = success)
  #  */
  # int8_t - uint8_t devAddr, uint8_t regAddr, uint8_t *data
  def readByte(self, devAddr, regAddr):
    return self.bus.read_byte_data(devAddr, regAddr)

  # /** Read multiple bytes from an 8-bit device register.
  #  * @param devAddr I2C slave device address
  #  * @param regAddr First register regAddr to read from
  #  * @param length Number of bytes to read
  #  * @param data Buffer to store read data in
  #  * @return I2C_TransferReturn_TypeDef http://downloads.energymicro.com/documentation/doxygen/group__I2C.html
  #  */
  # int8_t - uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data
  # def readBytes(devAddr, regAddr, length):
  #   bcm2835_i2c_setSlaveAddress(devAddr);
  #   sendBuf[0] = regAddr;
  #   uint8_t response = bcm2835_i2c_write_read_rs(sendBuf, 1, recvBuf, length);
  #   int i ;
  #   for (i = 0; i < length ; i++) {
  #     data[i] = (uint8_t) recvBuf[i];
  #   }
  #   return response == BCM2835_I2C_REASON_OK;
  def readBytes(self, devAddr, a_address, a_length):
    # if a_length > len(a_data_list):
    #   print('read_bytes, length of passed list too short')
    #   return a_data_list
    # Attempt to use the built in read bytes function in the adafruit lib
    # a_data_list = self.__bus.read_i2c_block_data(self.__dev_id, a_address,
    #                                             a_length)
    # Attempt to bypass adafruit lib
    #a_data_list = self.__mpu.bus.read_i2c_block_data(0x68, a_address, a_length)
    #print('data' + str(a_data_list))
    a_data_list = list()
    for x in range(0, a_length):
        # print("x:{}".format(x))
        a_data_list.append(self.bus.read_byte_data(devAddr, a_address + x))
    return a_data_list
  
  
  # /** Read single word from a 16-bit device register.
  #  * @param devAddr I2C slave device address
  #  * @param regAddr Register regAddr to read from
  #  * @param data Container for word value read from device
  #  * @return Status of read operation (true = success)
  #  */
  # int8_t - uint8_t devAddr, uint8_t regAddr, uint16_t *data
  # def readWord(self, devAddr, regAddr):
  #   return self.bus.read_word_data(devAddr, regAddr)

  # /** Read multiple words from a 16-bit device register.
  #  * @param devAddr I2C slave device address
  #  * @param regAddr First register regAddr to read from
  #  * @param length Number of words to read
  #  * @param data Buffer to store read data in
  #  * @return Number of words read (-1 indicates failure)
  #  */
  # int8_t 
  # def readWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data):
  #   bcm2835_i2c_setSlaveAddress(devAddr);
  #   sendBuf[0] = regAddr;
  #   uint8_t response = bcm2835_i2c_write_read_rs(sendBuf, 1, recvBuf, length*2 );
  #   uint8_t i;
  #   for (i = 0; i < length; i++) {
  #     data[i] = (recvBuf[i*2] << 8) | recvBuf[i*2+1] ;
  #   }
  #   return  response == BCM2835_I2C_REASON_OK ;

  # /** write a single bit in an 8-bit device register.
  #  * @param devAddr I2C slave device address
  #  * @param regAddr Register regAddr to write to
  #  * @param bitNum Bit position to write (0-7)
  #  * @param value New bit value to write
  #  * @return Status of operation (true = success)
  #  */
  # bool - uint8_t devAddr, uint8_t regAddr, uint8_t bitNum, uint8_t data
  def writeBit(self, devAddr, regAddr, bitNum, data):
    prev_data = self.bus.read_byte_data(devAddr, regAddr)
    next_data = 0
    if data != 0:
      next_data = (prev_data | (1 << bitNum))
    else:
      next_data = (prev_data & ~(1 << bitNum))
    self.bus.write_byte_data(devAddr, regAddr, next_data)
    # self.bus.write_byte_data(devAddr, regAddr, ctypes.c_int8(next_data).value)
  
  # def write_bit(self, devAddr, a_reg_add, a_bit_num, a_bit):
  #   byte = self.bus.read_byte_data(self.__dev_id, a_reg_add)
  #   if a_bit:
  #       byte |= 1 << a_bit_num
  #   else:
  #       byte &= ~(1 << a_bit_num)
  #   self.bus.write_byte_data(
  #       self.__dev_id, a_reg_add, ctypes.c_int8(byte).value)
    
      
    

  # /** Write multiple bits in an 8-bit device register.
  #  * @param devAddr I2C slave device address
  #  * @param regAddr Register regAddr to write to
  #  * @param bitStart First bit position to write (0-7)
  #  * @param length Number of bits to write (not more than 8)
  #  * @param data Right-aligned value to write
  #  * @return Status of operation (true = success)
  #  */
  # bool 
  # def writeBits(uint8_t devAddr, uint8_t regAddr, uint8_t bitStart, uint8_t length, uint8_t data):
  #   # //      010 value to write
  #   # // 76543210 bit numbers
  #   # //    xxx   args: bitStart=4, length=3
  #   # // 00011100 mask byte
  #   # // 10101111 original value (sample)
  #   # // 10100011 original & ~mask
  #   # // 10101011 masked | value
  #   bcm2835_i2c_setSlaveAddress(devAddr);
  #   # //first reading registery value
  #   sendBuf[0] = regAddr;
  #   uint8_t response = bcm2835_i2c_write_read_rs(sendBuf, 1, recvBuf, 1 );
  #   if ( response == BCM2835_I2C_REASON_OK ) {
  #     uint8_t b = recvBuf[0];
  #     uint8_t mask = ((1 << length) - 1) << (bitStart - length + 1);
  #     data <<= (bitStart - length + 1); // shift data into correct position
  #     data &= mask; // zero all non-important bits in data
  #     b &= ~(mask); // zero all important bits in existing byte
  #     b |= data; // combine data with existing byte
  #     sendBuf[1] = b ;
  #     response = bcm2835_i2c_write(sendBuf, 2);
  #     }
  #   return response == BCM2835_I2C_REASON_OK;
    
  def writeBits(self, devAddr, a_reg_add, a_bit_start, a_length, a_data):
    byte = self.bus.read_byte_data(devAddr, a_reg_add)
    mask = ((1 << a_length) - 1) << (a_bit_start - a_length + 1)
    # Get data in position and zero all non-important bits in data
    a_data <<= a_bit_start - a_length + 1
    a_data &= mask
    # Clear all important bits in read byte and combine with data
    byte &= ~mask
    byte = byte | a_data
    # Write the data to the I2C device
    # self.__bus.write_byte_data(self.__dev_id, a_reg_add, ctypes.c_int8(byte).value)
    self.bus.write_byte_data(devAddr, a_reg_add, byte)

  # /** Write single byte to an 8-bit device register.
  #  * @param devAddr I2C slave device address
  #  * @param regAddr Register address to write to
  #  * @param data New byte value to write
  #  * @return Status of operation (true = success)
  #  */
  # bool - uint8_t devAddr, uint8_t regAddr, uint8_t data
  def writeByte(self, devAddr, regAddr, data):
    self.bus.write_byte_data(devAddr, regAddr, data)

  # bool 
  # def writeBytes(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint8_t *data):
  #   bcm2835_i2c_setSlaveAddress(devAddr);
  #   sendBuf[0] = regAddr;
  #   uint8_t i;
  #   for (i = 0; i < length; i++) {
  #     sendBuf[i+1] = data[i] ;
  #   }
  #   uint8_t response = bcm2835_i2c_write(sendBuf, 1+length);
  #   return response == BCM2835_I2C_REASON_OK ;
  
  # bool - uint8_t devAddr, uint8_t regAddr, uint16_t data
  def writeWord(self, devAddr, regAddr, data):
    self.bus.write_word_data(devAddr, regAddr, data)

  # bool 
  # def writeWords(uint8_t devAddr, uint8_t regAddr, uint8_t length, uint16_t *data):
  #   bcm2835_i2c_setSlaveAddress(devAddr);
  #   sendBuf[0] = regAddr;
  #   uint8_t i;
  #   for (i = 0; i < length; i++) {
  #     sendBuf[1+2*i] = (uint8_t) (data[i] >> 8); //MSByte
  #     sendBuf[2+2*i] = (uint8_t) (data[i] >> 0); //LSByte
  #   }
  #   uint8_t response = bcm2835_i2c_write(sendBuf, 1+2*length);
  #   return response == BCM2835_I2C_REASON_OK ;
