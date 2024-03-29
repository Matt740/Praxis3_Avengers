from machine import I2C, Pin
from utime import sleep_ms

# I2C RELEVANT CLASSES
###############################################################################################################
#Defines I2C base commands for reading and writing bytes to sensors
class I2CBase:
    def writeto_mem(self, addr, memaddr, buf, *, addrsize=8):
        raise NotImplementedError('writeto_mem')

    def readfrom_mem(self, addr, memaddr, nbytes, *, addrsize=8):
        raise NotImplementedError('readfrom_mem')

    def write8(self, addr, buf, stop=True):
        raise NotImplementedError('write')

    def read16(self, addr, nbytes, stop=True):
        raise NotImplementedError('read')

    def __init__(self, bus=None, freq=None, sda=None, scl=None):
        raise NotImplementedError('__init__')

#Class which defines the I2C connection and which pins are being used using. Uses I2C base commands.
class I2CUnifiedMachine(I2CBase):
    def __init__(self, bus=None, freq=None, sda=None, scl=None):
        if freq is None: freq = 400_000
        if not isinstance(freq, (int)):
            raise ValueError("freq must be an Int")
        if freq < 400_000: print("\033[91mWarning: minimum freq 400kHz is recommended if using OLED module.\033[0m")
        if bus is not None and sda is not None and scl is not None:
            print('Using supplied bus, sda, and scl to create machine.I2C() with freq: {} Hz'.format(freq))
            self.i2c = I2C(bus, freq=freq, sda=sda, scl=scl)
        elif bus is None and sda is None and scl is None:
            self.i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=freq) # RPI Pico in Expansion Board
        else:
            raise Exception("Please provide at least bus, sda, and scl")

        self.writeto_mem = self.i2c.writeto_mem
        self.readfrom_mem = self.i2c.readfrom_mem

    def write8(self, addr, reg, data):
        if reg is None:
            self.writeto_mem(addr, data)
        else:
            self.writeto_mem(addr, reg, data)
            
    def read16(self, addr, reg):
        self.writeto_mem(addr, reg, False)
        return self.readfrom_mem(addr, 2)
        
    def scan(self):
        print([hex(i) for i in self.i2c.scan()])

# Overall function to create 12C object and returns the object 
def create_unified_i2c(bus=None, freq=None, sda=None, scl=None, suppress_warnings=True):
    i2c = I2CUnifiedMachine(bus=bus, freq=freq, sda=sda, scl=scl)
    return i2c

###############################################################################################################################

##############################################################################################################################################

# This code block is inspired by the code library provided by SparkFun Electornics https://github.com/sparkfun/Qwiic_KX13X_Py/blob/main/qwiic_kx13x.py#L58
# The class is reversed engineered from their code and the following datasheet https://cdn.sparkfun.com/assets/9/9/a/5/6/KX132-1211-Technical-Reference-Manual-Rev-1.0.pdf?_gl=1*1qax5u2*_ga*MTIyNDQ0NTAzOC4xNzA1MDkyMDQx*_ga_T369JS7J9N*MTcwOTc0NzE1NS40LjAuMTcwOTc0NzE1NS42MC4wLjA.
# We simplified their implementation drastically for our purposes and attmepted to make it further understandbale and made it integrable with our I2C classes

KX13X_I2C_ADDRESS = 0x1F

#Accelerometer
class QwiicKX13X(object):

    # Range and coversion factors
    KX132_WHO_AM_I = 0x3D
    KX132_RANGE2G  = 0x00
    KX132_RANGE4G  = 0x01
    KX132_RANGE8G  = 0x02
    KX132_RANGE16G = 0x03
    CONV_2G =  .00006103518784142582
    CONV_4G =  .0001220703756828516
    CONV_8G =  .0002441407513657033
    CONV_16G = .0004882811975463118
    TOTAL_ACCEL_DATA_16BIT = 6
    TOTAL_ACCEL_DATA_8BIT  = 3

    #Significant bits
    XLSB = 0
    XMSB = 1
    YLSB = 2
    YMSB = 3
    ZLSB = 4
    ZMSB = 5

    # Customization registers
    KX13X_CNTL1            = 0x1B
    KX13X_CNTL2            = 0x1C
    KX13X_CNTL3            = 0x1D
    KX13X_CNTL4            = 0x1E
    KX13X_CNTL5            = 0x1F
    KX13X_CNTL6            = 0x20
    KX13X_ODCNTL           = 0x21
    KX13X_INC1             = 0x22
    KX13X_INC2             = 0x23
    KX13X_INC3             = 0x24
    KX13X_INC4             = 0x25
    KX13X_INC5             = 0x26
    KX13X_INC6             = 0x27
    KX13X_XOUT_L           = 0x08


    def __init__(self, bus=None, freq=None, sda=None, scl=None, network=None, addr=KX13X_I2C_ADDRESS):
        #Attach sensor to I2C connection
        if network == None:
            self.i2c = create_unified_i2c(bus=bus, freq=freq, sda=sda, scl=scl)
        else:
            self.i2c = network
        self.addr = addr

    def set_power_mode(self, enable):
        if enable != True and enable != False:
            return Exception("Invalid power mode")
        
        reg_val = self.i2c.readfrom_mem(self.addr, self.KX13X_CNTL1,1)
        reg_val &= 0x7F
        reg_val |= (enable << 7)
        self.i2c.writeto_mem(self.addr, self.KX13X_CNTL1 , reg_val)

    def get_power_mode(self):
        reg_val = self.i2c.readfrom_mem(self.addr, self.KX13X_CNTL1, 1)
        if ((reg_val & 0x80) >> 7) == 1:
            return "ON"
        else:
            return "OFF"

    def initialize(self, performance=False, hardware_interupt= False, G_range=2, software_interupts=False, tilt=False):
        ''''Note for our purposes we should not need hardware_interupt or software interupt so please leave as false during 
            initialization
        '''
        #force restart
        self.set_power_mode(False)
        
        # Get specific wanted settings
        reg_value = 0x02 | performance
        reg_value = (reg_value << 1) | hardware_interupt
        reg_value = reg_value << 2

        if G_range == 2:
            tmp = 0x00
        elif G_range == 4:
            tmp = 0x01
        elif G_range == 8:
            tmp = 0x02
        elif G_range == 16:
            tmp = 0x03
        else:
            raise Exception("G range is invalid for sensor please enter 2, 4, 8, or 16")
        
        reg_value = reg_value | tmp
        reg_value = (reg_value << 1) | software_interupts
        reg_value = reg_value << 1
        reg_value = (reg_value << 1) | tilt

        #Send initialization to control register
        self.i2c.writeto_mem(self.addr, self.KX13X_CNTL1, reg_value)

    def set_g_range(self, G_range):
        reg_val = self.i2c.readfrom_mem(self.addr, self.KX13X_CNTL1, 1)
        reg_val &= 0xE7
        
        if G_range == 2:
            reg_val |= 0x00
        elif G_range == 4:
            reg_val |= 0x08
        elif G_range == 8:
            reg_val |= 0x10
        elif G_range == 16:
            reg_val |= 0x18
        else:
            raise Exception("G range is invalid for sensor please enter 2, 4, 8, or 16")
        self.i2c.writeto_mem(self.addr, self.KX13X_CNTL1 , reg_val)

    def get_raw_accel_values(self):

        reg_val = self.i2c.readfrom_mem(self.addr, self.KX13X_INC4, 1)
        if reg_val & 0x40:
            accel_data = self.i2c.readfrom_mem(self.addr, self.KX13X_XOUT_L, self.TOTAL_ACCEL_DATA_16BIT)
            xData = (accel_data[self.XMSB] << 8) | accel_data[self.XLSB]
            yData = (accel_data[self.YMSB] << 8) | accel_data[self.YLSB]
            zData = (accel_data[self.ZMSB] << 8) | accel_data[self.ZLSB]
        else:
            accel_data = self.i2c.readfrom_mem(self.addr, self.KX13X_XOUT_L, self.TOTAL_ACCEL_DATA_8BIT)
            xData = accel_data[0]
            yData = accel_data[1]
            zData = accel_data[2]

        self.raw_output_datax = xData
        self.raw_output_datay = yData
        self.raw_output_dataz = zData

    def conv_raw_accel_values(self):
        accel_range = self.i2c.readfrom_mem(self.addr, self.KX13X_CNTL1, 1)
        accel_range &= 0x18
        accel_range = accel_range >> 3

        if accel_range == self.KX132_RANGE2G:
            self.accelx = round(self.raw_output_datax * self.CONV_2G, 6)
            self.accely = round(self.raw_output_datay * self.CONV_2G, 6)
            self.accelz = round(self.raw_output_dataz * self.CONV_2G, 6)
        elif accel_range == self.KX132_RANGE4G:
            self.accelx = round(self.raw_output_datax * self.CONV_4G, 6)
            self.accely = round(self.raw_output_datay * self.CONV_4G, 6)
            self.accelz = round(self.raw_output_dataz * self.CONV_4G, 6)
        elif accel_range == self.KX132_RANGE8G:
            self.accelx = round(self.raw_output_datax * self.CONV_8G, 6)
            self.accely = round(self.raw_output_datay * self.CONV_8G, 6)
            self.accelz = round(self.raw_output_dataz * self.CONV_8G, 6)
        elif accel_range == self.KX132_RANGE16G:
            self.accelx = round(self.raw_output_datax * self.CONV_16G, 6)
            self.accely = round(self.raw_output_datay * self.CONV_16G, 6)
            self.accelz = round(self.raw_output_dataz * self.CONV_16G, 6)

    def get_accel_data(self):
        self.get_raw_accel_values()
        self.conv_raw_accel_values()
        return self.accelx, self.accely, self.accelz

##############################################################################################################################################
##############################################################################################################################################

#MATT YOUR SENSOR CODE HERE




































#######################################################################################################################################
#######################################################################################################################################















































#######################################################################################################################################