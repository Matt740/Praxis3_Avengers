import smbus2
import time
import serial
import RPi.GPIO as GPIO

# I2C RELEVANT CLASSES
###############################################################################################################
#Defines I2C base commands for reading and writing bytes to sensors
class I2CBase:
    def write_byte(self, i2c_addr, reg=None, data=0x00, force=None):
        raise NotImplementedError('write_byte')
        
    def read_byte(self, i2c_addr, reg=None, force=None):
        raise NotImplementedError('read_byte')
            
    def read_byte_block(self, i2c_addr, reg, length, force=None):
        raise NotImplementedError('read_byte_block')

    def write_byte_block(self, i2c_addr, reg, data=0x00, force=None):
        raise NotImplementedError('write_byte_block')

    def __init__(self, bus=0):
        raise NotImplementedError('__init__')

#Class which defines the I2C connection and which pins are being used using. Uses I2C base commands.
class I2CUnifiedMachine(I2CBase):
    def __init__(self, bus=0):
        if bus != 0 and bus != 1:
            raise Exception("Please provide valid bus number")
        else:
            self.i2c = smbus2.SMBus(bus)  #Bus 1 is typically used bus

    def write_byte(self, i2c_addr, reg=None, data=0x00, force=None):
        if reg == None:
            return self.i2c.write_byte(i2c_addr, data, force)
        else:
            return self.i2c.write_byte_data(i2c_addr, reg, data, force)
        
    def read_byte(self, i2c_addr, reg=None, force=None):
        if reg == None:
            return self.i2c.read_byte(i2c_addr, force)
        else:
            return self.i2c.read_byte_data(i2c_addr, reg, force)
            
    def read_byte_block(self, i2c_addr, reg, length, force=None):
        return self.i2c.read_i2c_block_data(i2c_addr, reg, length, force)

    def write_byte_block(self, i2c_addr, reg, data=0x00, force=None):
        return self.i2c.write_i2c_block_data(i2c_addr, reg, data, force)
    
    def _get_regs(self, register, length):
        """Get one or more registers."""
        if length == 1:
            return self.i2c.read_byte(self.i2c_addr, register)
        else:
            return self.i2c.read_byte_block(self.i2c_addr, register, length)
        
    def _set_regs(self, register, value):
        """Set one or more registers."""
        if isinstance(value, int):
            self.i2c.write_byte_(self.i2c_addr, register, value)
        else:
            self.i2c.write_byte_block(self.i2c_addr, register, value)

# Overall function to create 12C object and returns the object 
def create_unified_i2c(bus=0):
    i2c = I2CUnifiedMachine(bus=bus)
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


    def __init__(self, bus=None, network=None, addr=KX13X_I2C_ADDRESS):
        #Attach sensor to I2C connection
        if network == None:
            self.i2c = create_unified_i2c(bus=bus)
        else:
            self.i2c = network
        self.addr = addr

    def set_power_mode(self, enable):
        if enable != True and enable != False:
            return Exception("Invalid power mode")
        
        reg_val = self.i2c.read_byte(self.addr, self.KX13X_CNTL1)  
        reg_val &= 0x7F
        reg_val |= (enable << 7)
        self.i2c.write_byte(self.addr, self.KX13X_CNTL1 , reg_val)

    def get_power_mode(self):
        reg_val = self.i2c.read_byte(self.addr, self.KX13X_CNTL1)
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
        self.i2c.write_byte(self.addr, self.KX13X_CNTL1, reg_value)

    def set_g_range(self, G_range):
        reg_val = self.i2c.read_byte(self.addr, self.KX13X_CNTL1)
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
        self.i2c.write_byte(self.addr, self.KX13X_CNTL1 , reg_val)

    def get_raw_accel_values(self):

        reg_val = self.i2c.read_byte(self.addr, self.KX13X_INC4) #These next two lines sketch me out
        if reg_val & 0x40:
            accel_data = self.i2c.read_byte_block(self.addr, self.KX13X_XOUT_L, self.TOTAL_ACCEL_DATA_16BIT)
            xData = (accel_data[self.XMSB] << 8) | accel_data[self.XLSB]
            yData = (accel_data[self.YMSB] << 8) | accel_data[self.YLSB]
            zData = (accel_data[self.ZMSB] << 8) | accel_data[self.ZLSB]
        else:
            accel_data = self.i2c.read_byte_block(self.addr, self.KX13X_XOUT_L, self.TOTAL_ACCEL_DATA_8BIT)
            xData = accel_data[0]
            yData = accel_data[1]
            zData = accel_data[2]

        self.raw_output_datax = xData
        self.raw_output_datay = yData
        self.raw_output_dataz = zData

    def conv_raw_accel_values(self):
        accel_range = self.i2c.read_byte(self.addr, self.KX13X_CNTL1)
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

BME680_I2C_ADDRESS = 0x76

#Accelerometer
class BME680(object):

    #Todo
    #Change constants
    #Update I2C functions

    SLEEP_MODE = 0
    FORCED_MODE = 1
    CONF_T_P_MODE_ADDR = 0x74
    MODE_MSK = 0x03
    MODE_POS = 0
    POLL_PERIOD_MS = 10
    CONF_T_P_MODE_ADDR = 0x74
    FIELD0_ADDR = 0x1d
    NEW_DATA_MSK = 0x80
    FIELD_LENGTH = 17
    GAS_INDEX_MSK = 0x0f
    COEFF_ADDR1 = 0x89
    COEFF_ADDR2 = 0xe1
    COEFF_ADDR1_LEN = 25
    COEFF_ADDR2_LEN = 16
    ADDR_RES_HEAT_VAL_ADDR = 0x00
    ADDR_RES_HEAT_RANGE_ADDR = 0x02
    ADDR_RANGE_SW_ERR_ADDR = 0x04
    CONF_ODR_FILT_ADDR = 0x75
    FILTER_MSK = 0X1C
    FILTER_POS = 2
    OSP_MSK = 0X1C
    OSP_POS = 2

    def __init__(self, bus=None, network=None, addr=BME680_I2C_ADDRESS):
        #Attach sensor to I2C connection
        if network == None:
            self.i2c = create_unified_i2c(bus=bus)
        else:
            self.i2c = network
        self.addr = addr

        self.soft_reset()
        self.set_power_mode(constants.SLEEP_MODE)

        self._get_calibration_data()

        #self.set_humidity_oversample(constants.OS_2X)
        self.set_pressure_oversample(constants.OS_4X)
        #self.set_temperature_oversample(constants.OS_8X)
        self.set_filter(constants.FILTER_SIZE_3)
        #if self._variant == constants.VARIANT_HIGH:
        #    self.set_gas_status(constants.ENABLE_GAS_MEAS_HIGH)
        #else:
        #    self.set_gas_status(constants.ENABLE_GAS_MEAS_LOW)
        self.set_temp_offset(0)
        self.get_sensor_data()
    
    

    def set_power_mode(self, value, blocking=True):
        """Set power mode."""
        if value not in (self.SLEEP_MODE, self.FORCED_MODE):
            raise ValueError('Power mode should be one of SLEEP_MODE or FORCED_MODE')

        self.power_mode = value

        self.i2c._get_regs(self.CONF_T_P_MODE_ADDR, self.MODE_MSK, self.MODE_POS, value)

        while blocking and self.get_power_mode() != self.power_mode:
            time.sleep(self.POLL_PERIOD_MS / 1000.0)

    def get_power_mode(self):
        """Get power mode."""
        self.power_mode = self.i2c._get_regs(self.CONF_T_P_MODE_ADDR, 1)
        return self.power_mode

    def _get_calibration_data(self):
        """Retrieve the sensor calibration data and store it in .calibration_data."""
        calibration = self.i2c._get_regs(self.COEFF_ADDR1, self.COEFF_ADDR1_LEN)
        calibration += self.i2c._get_regs(self.COEFF_ADDR2, self.COEFF_ADDR2_LEN)

        heat_range = self.i2c._get_regs(self.ADDR_RES_HEAT_RANGE_ADDR, 1)
        heat_value = self.twos_comp(self.i2c._get_regs(self.ADDR_RES_HEAT_VAL_ADDR, 1), bits=8)
        sw_error = self.twos_comp(self.i2c._get_regs(self.ADDR_RANGE_SW_ERR_ADDR, 1), bits=8)

        self.calibration_data.set_from_array(calibration)
        self.calibration_data.set_other(heat_range, heat_value, sw_error)

    def twos_comp(val, bits=16):
        """Convert two bytes into a two's compliment signed word."""
        # TODO: Reimpliment with struct
        if val & (1 << (bits - 1)) != 0:
            val = val - (1 << bits)
        return val

    def get_sensor_data(self):
        """Get sensor data.

        Stores data in .data and returns True upon success.

        """
        self.set_power_mode(self.FORCED_MODE)

        for attempt in range(10):
            status = self.i2c._get_regs(self.FIELD0_ADDR, 1)

            if (status & self.NEW_DATA_MSK) == 0:
                time.sleep(self.POLL_PERIOD_MS / 1000.0)
                continue

            regs = self.i2c._get_regs(self.FIELD0_ADDR, self.FIELD_LENGTH)

            self.data.status = regs[0] & self.NEW_DATA_MSK
            # Contains the nb_profile used to obtain the current measurement
            #self.data.gas_index = regs[0] & self.GAS_INDEX_MSK
            self.data.meas_index = regs[1]

            adc_pres = (regs[2] << 12) | (regs[3] << 4) | (regs[4] >> 4)
            #adc_temp = (regs[5] << 12) | (regs[6] << 4) | (regs[7] >> 4)
            #adc_hum = (regs[8] << 8) | regs[9]
            #adc_gas_res_low = (regs[13] << 2) | (regs[14] >> 6)
            #adc_gas_res_high = (regs[15] << 2) | (regs[16] >> 6)
            #gas_range_l = regs[14] & constants.GAS_RANGE_MSK
            #gas_range_h = regs[16] & constants.GAS_RANGE_MSK

           # if self._variant == constants.VARIANT_HIGH:
            #    self.data.status |= regs[16] & constants.GASM_VALID_MSK
            #    self.data.status |= regs[16] & constants.HEAT_STAB_MSK
            #else:
            #    self.data.status |= regs[14] & constants.GASM_VALID_MSK
            #    self.data.status |= regs[14] & constants.HEAT_STAB_MSK

            #self.data.heat_stable = (self.data.status & constants.HEAT_STAB_MSK) > 0

            #temperature = self._calc_temperature(adc_temp)
            #self.data.temperature = temperature / 100.0
            #self.ambient_temperature = temperature  # Saved for heater calc

            self.data.pressure = self._calc_pressure(adc_pres) / 100.0
            #self.data.humidity = self._calc_humidity(adc_hum) / 1000.0

            #if self._variant == constants.VARIANT_HIGH:
            #    self.data.gas_resistance = self._calc_gas_resistance_high(adc_gas_res_high, gas_range_h)
            #else:
            #    self.data.gas_resistance = self._calc_gas_resistance_low(adc_gas_res_low, gas_range_l)

            return True

        return False
    
    def set_filter(self, value):
        """Set IIR filter size.

        Optionally remove short term fluctuations from the temperature and pressure readings,
        increasing their resolution but reducing their bandwidth.

        Enabling the IIR filter does not slow down the time a reading takes, but will slow
        down the BME680s response to changes in temperature and pressure.

        When the IIR filter is enabled, the temperature and pressure resolution is effectively 20bit.
        When it is disabled, it is 16bit + oversampling-1 bits.

        """
        self.tph_settings.filter = value
        self._set_bits(self.CONF_ODR_FILT_ADDR, self.FILTER_MSK, self.FILTER_POS, value)

    def get_filter(self):
        """Get filter size."""
        return (self.i2c._get_regs(self.CONF_ODR_FILT_ADDR, 1) & self.FILTER_MSK) >> self.FILTER_POS

    
    def set_pressure_oversample(self, value):
        """Set temperature oversampling.

        A higher oversampling value means more stable sensor readings,
        with less noise and jitter.

        However each step of oversampling adds about 2ms to the latency,
        causing a slower response time to fast transients.

        :param value: Oversampling value, one of: OS_NONE, OS_1X, OS_2X, OS_4X, OS_8X, OS_16X

        """
        self.tph_settings.os_pres = value
        self.i2c._set_bits(self.CONF_T_P_MODE_ADDR, self.OSP_MSK, self.OSP_POS, value)

    def get_pressure_oversample(self):
        """Get pressure oversampling."""
        return (self.i2c._get_regs(self.CONF_T_P_MODE_ADDR, 1) & self.OSP_MSK) >> self.OSP_POS

    def _calc_pressure(self, pressure_adc):
        """Convert the raw pressure using calibration data."""
        var1 = ((self.calibration_data.t_fine) >> 1) - 64000
        var2 = ((((var1 >> 2) * (var1 >> 2)) >> 11) *
                self.calibration_data.par_p6) >> 2
        var2 = var2 + ((var1 * self.calibration_data.par_p5) << 1)
        var2 = (var2 >> 2) + (self.calibration_data.par_p4 << 16)
        var1 = (((((var1 >> 2) * (var1 >> 2)) >> 13) *
                ((self.calibration_data.par_p3 << 5)) >> 3) +
                ((self.calibration_data.par_p2 * var1) >> 1))
        var1 = var1 >> 18

        var1 = ((32768 + var1) * self.calibration_data.par_p1) >> 15
        calc_pressure = 1048576 - pressure_adc
        calc_pressure = ((calc_pressure - (var2 >> 12)) * (3125))

        if calc_pressure >= (1 << 31):
            calc_pressure = ((calc_pressure // var1) << 1)
        else:
            calc_pressure = ((calc_pressure << 1) // var1)

        var1 = (self.calibration_data.par_p9 * (((calc_pressure >> 3) *
                (calc_pressure >> 3)) >> 13)) >> 12
        var2 = ((calc_pressure >> 2) *
                self.calibration_data.par_p8) >> 13
        var3 = ((calc_pressure >> 8) * (calc_pressure >> 8) *
                (calc_pressure >> 8) *
                self.calibration_data.par_p10) >> 17

        calc_pressure = (calc_pressure) + ((var1 + var2 + var3 +
                                           (self.calibration_data.par_p7 << 7)) >> 4)

        return calc_pressure


    




















##############################################################################################################################################
##############################################################################################################################################

UART_PIN = '/dev/ttyS0'
Temp = '0123456789ABCDEF*'

class L76B(object):

    #Baud rate
    SET_NMEA_BAUDRATE          = '$PMTK251'
    SET_NMEA_BAUDRATE_115200   = 115200
    SET_NMEA_BAUDRATE_57600    = 57600
    SET_NMEA_BAUDRATE_38400    = 38400
    SET_NMEA_BAUDRATE_19200    = 19200
    SET_NMEA_BAUDRATE_14400    = 14400
    SET_NMEA_BAUDRATE_9600     = 9600
    SET_NMEA_BAUDRATE_4800     = 4800

    def __init__(self, StandByPin, ForcePin):
        self.ser = serial.Serial(port=UART_PIN, 
                                 baudrate=9600, 
                                 parity=serial.PARITY_NONE, 
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS,
                                 timeout=1000)
        GPIO.setmode(GPIO.BCM)
        GPIO.setup(StandByPin, GPIO.OUT)
        GPIO.setup(ForcePin, GPIO.OUT)
        GPIO.output(StandByPin, GPIO.LOW)
        GPIO.output(ForcePin, GPIO.LOW)

    def l76x_send_command(self, data): 
        Check = ord(data[1]) 
        for i in range(2, len(data)):
            Check = Check ^ ord(data[i]) 
        data = data + Temp[16]
        data = data + Temp[int(Check/16)]
        data = data + Temp[int(Check%16)]
        self.uart_send_string(data.encode())
        self.uart_send_byte('\r'.encode())
        self.uart_send_byte('\n'.encode())
        print (data)   

    def l76x_exit_backup_mode(self): #NEED
        self.Force.value(1)
        time.sleep(1)
        self.Force.value(0)
        time.sleep(1)
        self.Force = Pin(self.FORCE_PIN,Pin.IN)
        
    def uart_send_byte(self, value): # Nre wrtie
        self.ser.write(value) 

    def uart_send_string(self, value): # rewrite 
        self.ser.write(value)

    def uart_receive_byte(self): # rewrite
        return self.ser.read(1)

    def uart_receiveString(self, value): # rewrite
        data = self.ser.read(value)
        return data
    
    def uart_any(self): # rewrite
        return self.ser.any() 

