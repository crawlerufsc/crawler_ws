import math
import ctypes
import time
from imu_interface_py.I2cdev import I2cdev
from imu_interface_py.MPU6050_cons import MPU6050_cons
import ctypes


CONS = MPU6050_cons()
dmpPacketSize = 0

class Quartenion:
    def __init__(self):
        self.w = 0.0
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

class XYZ_int:
    def __init__(self):
        self.x = 0
        self.y = 0
        self.z = 0

class XYZ_float:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.z = 0.0


class MPU6050:

    def __init__(self, a_bus=1, a_address=CONS.MPU6050_DEFAULT_ADDRESS):
        # self.bus = smbus.SMBus(a_bus)
        self.i2c = I2cdev(a_bus)
        self.devAddr = a_address
    
    # Power on and prepare for general usage.
    # This will activate the device and take it out of sleep mode (which must be done
    # after start-up). This function also sets both the accelerometer and the gyroscope
    # to their most sensitive settings, namely +/- 2g and +/- 250 degrees/sec, and sets
    # the clock source to use the X Gyro for reference, which is slightly better than
    # the default internal clock source.
    
    def initialize(self):
        self.setClockSource(CONS.MPU6050_CLOCK_PLL_XGYRO)
        self.setFullScaleGyroRange(CONS.MPU6050_GYRO_FS_250)
        self.setFullScaleAccelRange(CONS.MPU6050_ACCEL_FS_2)
        self.setSleepEnabled(False)
        print("Test Connection:{}".format(self.testConnection()))
        if not self.testConnection():
            awdawd
    
    # bool 
    def testConnection(self):
        return self.getDeviceID() == 0x34
    # uint8_t -
    def getDeviceID(self):
        return self.i2c.readBits(self.devAddr, CONS.MPU6050_RA_WHO_AM_I, CONS.MPU6050_WHO_AM_I_BIT, CONS.MPU6050_WHO_AM_I_LENGTH)
    
    # this is the most basic initialization I can create. with the intent that we access the register bytes as few times as needed to get the job done.
    # for detailed descriptins of all registers and there purpose google "MPU-6000/MPU-6050 Register Map and Descriptions"
    # uint8_t 
    def dmpInitialize(self): # Lets get it over with fast Write everything once and set it up necely
        global dmpPacketSize
        # // Reset procedure per instructions in the "MPU-6000/MPU-6050 Register Map and Descriptions" page 41
        self.i2c.writeBit(self.devAddr,0x6B, 7, 1) #//PWR_MGMT_1: reset with 100ms delay
        # // sleep_ms(100);
        # delay(100);
        time.sleep(0.2)
        self.i2c.writeBits(self.devAddr,0x6A, 2, 3, 0b111) #// full SIGNAL_PATH_RESET: with another 100ms delay
        # // sleep_ms(100);
        # delay(100);    
        time.sleep(0.2)     
        self.i2c.writeByte(self.devAddr,0x6B, 0x01) #// 1000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
        self.i2c.writeByte(self.devAddr,0x38, 0x00) #// 0000 0000 INT_ENABLE: no Interrupt
        self.i2c.writeByte(self.devAddr,0x23, 0x00) #// 0000 0000 MPU FIFO_EN: (all off) Using DMP's FIFO instead
        self.i2c.writeByte(self.devAddr,0x1C, 0x00) #// 0000 0000 ACCEL_CONFIG: 0 =  Accel Full Scale Select: 2g
        self.i2c.writeByte(self.devAddr,0x37, 0x80) #// 1001 0000 INT_PIN_CFG: ACTL The logic level for int pin is active low. and interrupt status bits are cleared on any read
        self.i2c.writeByte(self.devAddr,0x6B, 0x01) #// 0000 0001 PWR_MGMT_1: Clock Source Select PLL_X_gyro
        self.i2c.writeByte(self.devAddr,0x19, 0x04) #// 0000 0100 SMPLRT_DIV: Divides the internal sample rate 400Hz ( Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
        self.i2c.writeByte(self.devAddr,0x1A, 0x01) #// 0000 0001 CONFIG: Digital Low Pass Filter (DLPF) Configuration 188HZ  //Im betting this will be the beat
        if (not self.writeProgMemoryBlock(CONS.dmpMemory, CONS.MPU6050_DMP_CODE_SIZE)):
            return 1 #// Loads the DMP image into the MPU6050 Memory // Should Never Fail
        test_res = self.testWriteProgMemoryBlock(CONS.dmpMemory, CONS.MPU6050_DMP_CODE_SIZE)
        print("test_res = {}".format(test_res))
        self.i2c.writeWord(self.devAddr,0x70, 0x0400) #// DMP Program Start Address
        self.i2c.writeByte(self.devAddr,0x1B, 0x18)     #// 0001 1000 GYRO_CONFIG: 3 = +2000 Deg/sec
        # test
        test_data = self.i2c.readByte(self.devAddr,0x1B)
        if test_data != 0x18:
            print("Erro test data")
            qqqqqqqqq
        self.i2c.writeByte(self.devAddr,0x6A, 0xC0)     #// 1100 1100 USER_CTRL: Enable Fifo and Reset Fifo
        self.i2c.writeByte(self.devAddr,0x38, 0x02)     #// 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on
        
        # self.i2c.writeBytes(self.devAddr,0x6B, 1, &(val = 0x01)); #// 1000 0001 PWR_MGMT_1:Clock Source Select PLL_X_gyro
        # self.i2c.writeBytes(self.devAddr,0x38, 1, &(val = 0x00)); #// 0000 0000 INT_ENABLE: no Interrupt
        # self.i2c.writeBytes(self.devAddr,0x23, 1, &(val = 0x00)); #// 0000 0000 MPU FIFO_EN: (all off) Using DMP's FIFO instead
        # self.i2c.writeBytes(self.devAddr,0x1C, 1, &(val = 0x00)); #// 0000 0000 ACCEL_CONFIG: 0 =  Accel Full Scale Select: 2g
        # self.i2c.writeBytes(self.devAddr,0x37, 1, &(val = 0x80)); #// 1001 0000 INT_PIN_CFG: ACTL The logic level for int pin is active low. and interrupt status bits are cleared on any read
        # self.i2c.writeBytes(self.devAddr,0x6B, 1, &(val = 0x01)); #// 0000 0001 PWR_MGMT_1: Clock Source Select PLL_X_gyro
        # self.i2c.writeBytes(self.devAddr,0x19, 1, &(val = 0x04)); #// 0000 0100 SMPLRT_DIV: Divides the internal sample rate 400Hz ( Sample Rate = Gyroscope Output Rate / (1 + SMPLRT_DIV))
        # self.i2c.writeBytes(self.devAddr,0x1A, 1, &(val = 0x01)); #// 0000 0001 CONFIG: Digital Low Pass Filter (DLPF) Configuration 188HZ  //Im betting this will be the beat
        # if (not self.writeProgMemoryBlock(dmpMemory, CONS.MPU6050_DMP_CODE_SIZE)):
        #     return 1; #// Loads the DMP image into the MPU6050 Memory // Should Never Fail
        # self.i2c.writeWords(self.devAddr, 0x70, 1, &(ival = 0x0400)); #// DMP Program Start Address
        # self.i2c.writeBytes(self.devAddr,0x1B, 1, &(val = 0x18));     #// 0001 1000 GYRO_CONFIG: 3 = +2000 Deg/sec
        # self.i2c.writeBytes(self.devAddr,0x6A, 1, &(val = 0xC0));     #// 1100 1100 USER_CTRL: Enable Fifo and Reset Fifo
        # self.i2c.writeBytes(self.devAddr,0x38, 1, &(val = 0x02));     #// 0000 0010 INT_ENABLE: RAW_DMP_INT_EN on
       
        self.i2c.writeBit(self.devAddr,0x6A, 2, 1) #// Reset FIFO one last time just for kicks. (MPUi2cWrite reads 0x6A first and only alters 1 bit and then saves the byte)

        self.setDMPEnabled(False) #// disable DMP for compatibility with the MPU6050 library
        # /*
        #     dmpPacketSize += 16;//DMP_FEATURE_6X_LP_QUAT
        #     dmpPacketSize += 6;//DMP_FEATURE_SEND_RAW_ACCEL
        #     dmpPacketSize += 6;//DMP_FEATURE_SEND_RAW_GYRO
        # */
        dmpPacketSize = 28
        return 0
    
    # awd - bool 
    def setDMPEnabled(self, enabled):
        self.i2c.writeBit(self.devAddr, CONS.MPU6050_RA_USER_CTRL, CONS.MPU6050_USERCTRL_DMP_EN_BIT, enabled)
    
    # uint8_t 
    def getIntStatus(self):
        return self.i2c.readByte(self.devAddr, CONS.MPU6050_RA_INT_STATUS)

    # uint16_t 
    def dmpGetFIFOPacketSize(self):
        global dmpPacketSize
        return dmpPacketSize
    
    # /** Get current FIFO buffer size.
    # * This value indicates the number of bytes stored in the FIFO buffer. This
    # * number is in turn the number of bytes that can be read from the FIFO buffer
    # * and it is directly proportional to the number of samples available given the
    # * set of sensor data bound to be stored in the FIFO (register 35 and 36).
    # * @return Current FIFO buffer size
    # */
    # uint16_t 
    def getFIFOCount(self):
        # print("getFIFOCount")
        buffer = self.i2c.readBytes(self.devAddr, CONS.MPU6050_RA_FIFO_COUNTH, 2)
        # if(buffer[0]!=0 or buffer[1]!=0):
        #     print("Fifocount = {}".format(((buffer[0]) << 8) | buffer[1]))
        return ((buffer[0]) << 8) | buffer[1]
    
    # /** Reset the FIFO.
    # * This bit resets the FIFO buffer when set to 1 while FIFO_EN equals 0. This
    # * bit automatically clears to 0 after the reset has been triggered.
    # * @see MPU6050_RA_USER_CTRL
    # * @see MPU6050_USERCTRL_FIFO_RESET_BIT
    # */
    # void 
    def resetFIFO(self):
        # self.i2c.writeBit(self.devAddr, CONS.MPU6050_RA_USER_CTRL, CONS.MPU6050_USERCTRL_FIFO_RESET_BIT, True)
        self.i2c.writeBit(self.devAddr, CONS.MPU6050_RA_USER_CTRL, CONS.MPU6050_USERCTRL_FIFO_RESET_BIT, 0b1)
    
    # // FIFO_R_W register
    # /** Get byte from FIFO buffer.
    # * This register is used to read and write data from the FIFO buffer. Data is
    # * written to the FIFO in order of register number (from lowest to highest). If
    # * all the FIFO enable flags (see below) are enabled and all External Sensor
    # * Data registers (Registers 73 to 96) are associated with a Slave device, the
    # * contents of registers 59 through 96 will be written in order at the Sample
    # * Rate.
    # *
    # * The contents of the sensor data registers (Registers 59 to 96) are written
    # * into the FIFO buffer when their corresponding FIFO enable flags are set to 1
    # * in FIFO_EN (Register 35). An additional flag for the sensor data registers
    # * associated with I2C Slave 3 can be found in I2C_MST_CTRL (Register 36).
    # *
    # * If the FIFO buffer has overflowed, the status bit FIFO_OFLOW_INT is
    # * automatically set to 1. This bit is located in INT_STATUS (Register 58).
    # * When the FIFO buffer has overflowed, the oldest data will be lost and new
    # * data will be written to the FIFO.
    # *
    # * If the FIFO buffer is empty, reading this register will return the last byte
    # * that was previously read from the FIFO until new data is available. The user
    # * should check FIFO_COUNT to ensure that the FIFO buffer is not read when
    # * empty.
    # *
    # * @return Byte from FIFO buffer
    # */
    # uint8_t 
    def getFIFOByte(self):
        return self.i2c.readByte(self.devAddr, CONS.MPU6050_RA_FIFO_R_W)
        

    # void - uint8_t *data, uint8_t length
    # def getFIFOBytes(self, length):
    #     if length > 0:
    #         return self.i2c.readBytes(self.devAddr, CONS.MPU6050_RA_FIFO_R_W, length)
    #     else:
    #         return []
    
    def getFIFOBytes(self, a_FIFO_count):
        return_list = list()
        for index in range(0, a_FIFO_count):
            return_list.append(
                self.i2c.bus.read_byte_data(self.devAddr, CONS.MPU6050_RA_FIFO_R_W))
        # print(return_list)
        return return_list
    
    # uint8_t - int16_t *data, const uint8_t* packet
    def dmpGetQuaternion2(self, a_fifo_bufer):
        #// TODO: accommodate different arrangements of sent data (ONLY default supported now)
        data = [0,0,0,0]
        # if packet == 0:
        #     packet = dmpPacketBuffer acho q nao faz nada dmpPacketBuffer nunca recebe nada
        data[0] = ctypes.c_int16((a_fifo_bufer[0] << 8) | a_fifo_bufer[1]).value 
        # ((a_fifo_bufer[0] << 8) | a_fifo_bufer[1])
        data[1] = ctypes.c_int16((a_fifo_bufer[4] << 8) | a_fifo_bufer[5]).value
        data[2] = ctypes.c_int16((a_fifo_bufer[8] << 8) | a_fifo_bufer[9]).value
        # ((a_fifo_bufer[8] << 8) | a_fifo_bufer[9])
        data[3] = ctypes.c_int16((a_fifo_bufer[12] << 8) | a_fifo_bufer[13]).value
        # ((a_fifo_bufer[12] << 8) | a_fifo_bufer[13])
        return data
    
    # uint8_t 
    def dmpGetQuaternion(self, a_fifo_bufer):
        #// TODO: accommodate different arrangements of sent data (ONLY default supported now)
        qI = self.dmpGetQuaternion2(a_fifo_bufer)
        q = Quartenion()
        q.w = qI[0] / 16384.0
        q.x = qI[1] / 16384.0
        q.y = qI[2] / 16384.0
        q.z = qI[3] / 16384.0
        return q
    
    # uint8_t - , VectorInt16 *v, const uint8_t* packet
    def dmpGetAccel(self, a_fifo_bufer):
        #// TODO: accommodate different arrangements of sent data (ONLY default supported now)
        # if (packet == 0) packet = dmpPacketBuffer;
        v = XYZ_int()
        v.x = ctypes.c_int16((a_fifo_bufer[16] << 8) | a_fifo_bufer[17]).value
        # (a_fifo_bufer[16] << 8) | a_fifo_bufer[17]
        v.y = ctypes.c_int16((a_fifo_bufer[18] << 8) | a_fifo_bufer[19]).value
        # (a_fifo_bufer[18] << 8) | a_fifo_bufer[19]
        v.z = ctypes.c_int16((a_fifo_bufer[20] << 8) | a_fifo_bufer[21]).value 
        # (a_fifo_bufer[20] << 8) | a_fifo_bufer[21]
        return v
    
    # uint8_t - , VectorFloat *v, Quaternion *q
    def dmpGetGravity(self, q):
        v = XYZ_float()
        v.x = 2 * (q.x * q.z - q.w * q.y)
        v.y = 2 * (q.w * q.x + q.y * q.z)
        v.z = q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z
        return v
    
    # uint8_t - float *data, Quaternion *q, VectorFloat *gravity
    def dmpGetYawPitchRoll(self, q, gravity):
        ypr = [0]*3
        #// yaw: (about Z axis)
        ypr[0] = math.atan2(2 * q.x * q.y - 2 * q.w * q.z, 2 * q.w * q.w + 2 * q.x * q.x - 1)
        #// pitch: (nose up/down, about Y axis)
        ypr[1] = math.atan2(gravity.x , math.sqrt(gravity.y * gravity.y + gravity.z * gravity.z))
        #// roll: (tilt left/right, about X axis)
        ypr[2] = math.atan2(gravity.y , gravity.z)
        if (gravity.z < 0):
            if(ypr[1] > 0):
                ypr[1] = math.pi - ypr[1]
            else:
                ypr[1] = -math.pi - ypr[1]
        return ypr
    
    # uint8_t - VectorInt16 *v, VectorInt16 *vRaw, VectorFloat *gravity
    def dmpGetLinearAccel(self, vRaw, gravity):
        #// get rid of the gravity component (+1g = +8192 in standard DMP FIFO packet, sensitivity is 2g)
        v = XYZ_int()
        v.x = ctypes.c_int16(int(vRaw.x - (gravity.x*8192))).value
        # vRaw.x - gravity.x * 8192
        v.y = ctypes.c_int16(int(vRaw.y - (gravity.y*8192))).value
        # vRaw.y - gravity.y * 8192
        v.z = ctypes.c_int16(int(vRaw.z - (gravity.z*8192))).value
        # vRaw.z - gravity.z * 8192
        return v
    
    # uint8_t - VectorInt16 *v, const uint8_t* packet
    def dmpGetGyro(self, a_fifo_bufer):
        #// TODO: accommodate different arrangements of sent data (ONLY default supported now)
        # if (packet == 0) packet = dmpPacketBuffer;
        v = XYZ_int()
        v.x = ctypes.c_int16((a_fifo_bufer[22] << 8) | a_fifo_bufer[23]).value
        # (a_fifo_bufer[22] << 8) | a_fifo_bufer[23]
        v.y = ctypes.c_int16((a_fifo_bufer[24] << 8) | a_fifo_bufer[25]).value
        # (a_fifo_bufer[24] << 8) | a_fifo_bufer[25]
        v.z = ctypes.c_int16((a_fifo_bufer[26] << 8) | a_fifo_bufer[27]).value
        # (a_fifo_bufer[26] << 8) | a_fifo_bufer[27]
        return v
        
    # /** Set clock source setting.
    # * An internal 8MHz oscillator, gyroscope based clock, or external sources can
    # * be selected as the MPU-60X0 clock source. When the internal 8 MHz oscillator
    # * or an external source is chosen as the clock source, the MPU-60X0 can operate
    # * in low power modes with the gyroscopes disabled.
    # *
    # * Upon power up, the MPU-60X0 clock source defaults to the internal oscillator.
    # * However, it is highly recommended that the device be configured to use one of
    # * the gyroscopes (or an external clock source) as the clock reference for
    # * improved stability. The clock source can be selected according to the following table:
    # *
    # * <pre>
    # * CLK_SEL | Clock Source
    # * --------+--------------------------------------
    # * 0       | Internal oscillator
    # * 1       | PLL with X Gyro reference
    # * 2       | PLL with Y Gyro reference
    # * 3       | PLL with Z Gyro reference
    # * 4       | PLL with external 32.768kHz reference
    # * 5       | PLL with external 19.2MHz reference
    # * 6       | Reserved
    # * 7       | Stops the clock and keeps the timing generator in reset
    # * </pre>
    # *
    # * @param source New clock source setting
    # * @see getClockSource()
    # * @see MPU6050_RA_PWR_MGMT_1
    # * @see MPU6050_PWR1_CLKSEL_BIT
    # * @see MPU6050_PWR1_CLKSEL_LENGTH
    # */
    # void - uint8_t 
    def setClockSource(self, source):
        self.i2c.writeBits(self.devAddr, CONS.MPU6050_RA_PWR_MGMT_1, CONS.MPU6050_PWR1_CLKSEL_BIT, CONS.MPU6050_PWR1_CLKSEL_LENGTH, source)

    # /** Set full-scale gyroscope range.
    # * @param range New full-scale gyroscope range value
    # * @see getFullScaleRange()
    # * @see MPU6050_GYRO_FS_250
    # * @see MPU6050_RA_GYRO_CONFIG
    # * @see MPU6050_GCONFIG_FS_SEL_BIT
    # * @see MPU6050_GCONFIG_FS_SEL_LENGTH
    # */
    # void - uint8_t range
    def setFullScaleGyroRange(self, length):
        self.i2c.writeBits(self.devAddr, CONS.MPU6050_RA_GYRO_CONFIG, CONS.MPU6050_GCONFIG_FS_SEL_BIT, CONS.MPU6050_GCONFIG_FS_SEL_LENGTH, length)
        
    # /** Set full-scale accelerometer range.
    # * @param range New full-scale accelerometer range setting
    # * @see getFullScaleAccelRange()
    # */
    # void - uint8_t range
    def setFullScaleAccelRange(self, length):
        self.i2c.writeBits(self.devAddr, CONS.MPU6050_RA_ACCEL_CONFIG, CONS.MPU6050_ACONFIG_AFS_SEL_BIT, CONS.MPU6050_ACONFIG_AFS_SEL_LENGTH, length)
    
    # /** Set sleep mode status.
    # * @param enabled New sleep mode enabled status
    # * @see getSleepEnabled()
    # * @see MPU6050_RA_PWR_MGMT_1
    # * @see MPU6050_PWR1_SLEEP_BIT
    # */
    # void - bool 
    def setSleepEnabled(self, enabled):
        self.i2c.writeBit(self.devAddr, CONS.MPU6050_RA_PWR_MGMT_1, CONS.MPU6050_PWR1_SLEEP_BIT, enabled)
        
    # bool -     const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify
    def writeProgMemoryBlock(self, data, dataSize, bank=0, address=0, verify=True):
        return self.writeMemoryBlock(data, dataSize, bank, address, verify)

    # bool - const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem
    # def writeMemoryBlock(self, const uint8_t *data, uint16_t dataSize, uint8_t bank, uint8_t address, bool verify, bool useProgMem):
    #     self.setMemoryBank(bank);
    #     self.setMemoryStartAddress(address);
    #     uint8_t chunkSize;
    #     uint8_t *verifyBuffer=0;
    #     uint8_t *progBuffer=0;
    #     uint16_t i;
    #     uint8_t j;
    #     if (verify) verifyBuffer = (uint8_t *)malloc(CONS.MPU6050_DMP_MEMORY_CHUNK_SIZE);
    #     if (useProgMem) progBuffer = (uint8_t *)malloc(CONS.MPU6050_DMP_MEMORY_CHUNK_SIZE);
    #     for (i = 0; i < dataSize;) {
    #         # // determine correct chunk size according to bank position and data size
    #         chunkSize = CONS.MPU6050_DMP_MEMORY_CHUNK_SIZE;

    #         # // make sure we don't go past the data size
    #         if (i + chunkSize > dataSize) chunkSize = dataSize - i;

    #         # // make sure this chunk doesn't go past the bank boundary (256 bytes)
    #         if (chunkSize > 256 - address) chunkSize = 256 - address;
            
    #         if (useProgMem) {
    #             # // write the chunk of data as specified
    #             # //for (j = 0; j < chunkSize; j++) progBuffer[j] = pgm_read_byte(data + i + j);
    #         } else {
    #             # // write the chunk of data as specified
    #             progBuffer = (uint8_t *)data + i;
    #         }

    #         self.i2c.writeBytes(self.devAddr, CONS.MPU6050_RA_MEM_R_W, chunkSize, progBuffer);

    #         # // verify data if needed
    #         if (verify && verifyBuffer) {
    #             self.setMemoryBank(bank);
    #             self.setMemoryStartAddress(address);
    #             self.i2c.readBytes(self.devAddr, CONS.MPU6050_RA_MEM_R_W, chunkSize, verifyBuffer);
    #             if (memcmp(progBuffer, verifyBuffer, chunkSize) != 0) {
    #                 # /*Serial.print("Block write verification error, bank ");
    #                 # Serial.print(bank, DEC);
    #                 # Serial.print(", address ");
    #                 # Serial.print(address, DEC);
    #                 # Serial.print("!\nExpected:");
    #                 # for (j = 0; j < chunkSize; j++) {
    #                 #     Serial.print(" 0x");
    #                 #     if (progBuffer[j] < 16) Serial.print("0");
    #                 #     Serial.print(progBuffer[j], HEX);
    #                 # }
    #                 # Serial.print("\nReceived:");
    #                 # for (uint8_t j = 0; j < chunkSize; j++) {
    #                 #     Serial.print(" 0x");
    #                 #     if (verifyBuffer[i + j] < 16) Serial.print("0");
    #                 #     Serial.print(verifyBuffer[i + j], HEX);
    #                 # }
    #                 # Serial.print("\n");*/
    #                 free(verifyBuffer);
    #                 if (useProgMem) free(progBuffer);
    #                 return false; #// uh oh.
    #             }
    #         }

    #         # // increase byte index by [chunkSize]
    #         i += chunkSize;

    #         # // uint8_t automatically wraps to 0 at 256
    #         address += chunkSize;

    #         # // if we aren't done, update bank (if necessary) and address
    #         if (i < dataSize) {
    #             if (address == 0) bank++;
    #             setMemoryBank(bank);
    #             setMemoryStartAddress(address);
    #         }
    #     }
    #     if (verify) free(verifyBuffer);
    #     if (useProgMem) free(progBuffer);
    #     return true;
    
    def writeMemoryBlock(self, a_data_list, a_data_size, a_bank, a_address,a_verify):
        success = True
        self.setMemoryBank(a_bank)
        self.setMemoryStartAddress(a_address)

        # For each a_data_item we want to write it to the board to a certain
        # memory bank and address
        for i in range(0, a_data_size):
            # Write each data to memory
            # print(i)
            self.i2c.bus.write_byte_data(self.devAddr, CONS.MPU6050_RA_MEM_R_W,a_data_list[i])
            # self.i2c.bus.write_byte(self.devAddr, CONS.MPU6050_RA_MEM_R_W,a_data_list[i])

            if a_verify:
                self.setMemoryBank(a_bank)
                self.setMemoryStartAddress(a_address)
                verify_data = self.i2c.bus.read_byte_data(self.devAddr,CONS.MPU6050_RA_MEM_R_W)
                # verify_data = self.i2c.bus.read_byte(self.devAddr,CONS.MPU6050_RA_MEM_R_W)
                if verify_data != a_data_list[i]:
                    # print("erro:i:{}".format(i))
                    # print("verify_data:{} -- a_data_list[i]:{}".format(verify_data,a_data_list[i]))
                    success = False

            # If we've filled the bank, change the memory bank
            if a_address == 255:
                a_address = 0
                a_bank += 1
                self.setMemoryBank(a_bank)
            else:
                a_address += 1

            # Either way update the memory address
            self.setMemoryStartAddress(a_address)

        return success
    
    def testWriteProgMemoryBlock(self, a_data_list, a_data_size, a_bank=0, a_address=0):
        success = True
        self.setMemoryBank(a_bank)
        self.setMemoryStartAddress(a_address)

        # For each a_data_item we want to write it to the board to a certain
        # memory bank and address
        for i in range(0, a_data_size):
            self.setMemoryBank(a_bank)
            self.setMemoryStartAddress(a_address)
            verify_data = self.i2c.bus.read_byte_data(self.devAddr,CONS.MPU6050_RA_MEM_R_W)

            if verify_data != a_data_list[i]:
                # print("erro:i:{}".format(i))
                # print("verify_data:{} -- a_data_list[i]:{}".format(verify_data,a_data_list[i]))
                success = False

            # If we've filled the bank, change the memory bank
            if a_address == 255:
                a_address = 0
                a_bank += 1
                self.setMemoryBank(a_bank)
            else:
                a_address += 1

            # Either way update the memory address
            self.setMemoryStartAddress(a_address)

        return success
    
    # void  - uint8_t bank, bool prefetchEnabled, bool userBank
    def setMemoryBank(self, bank, prefetchEnabled=False, userBank=False):
        bank &= 0x1F
        if (userBank):
            bank |= 0x20
        if (prefetchEnabled):
            bank |= 0x40
        # print("bank written:{}".format(bank))
        self.i2c.writeByte(self.devAddr, CONS.MPU6050_RA_BANK_SEL, bank)
        # read_bank = self.i2c.readByte(self.devAddr, CONS.MPU6050_RA_BANK_SEL)
        # print("bank read:{}".format(read_bank))

    # void - uint8_t address
    def setMemoryStartAddress(self, address):
        self.i2c.writeByte(self.devAddr, CONS.MPU6050_RA_MEM_START_ADDR, address)
