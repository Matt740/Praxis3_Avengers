import smbus2
import time
import math
import serial
import gpiozero as GPIO 
import pigpio
from picamera2 import Picamera2

#BME680 imports
from BME680_constants import lookupTable1, lookupTable2
from BME680_constants import BME680Data
import BME680_constants
import math

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
    
    def _get_regs(self, i2c_addr, register, length):
        """Get one or more registers."""
        if length == 1:
            return self.read_byte(i2c_addr, register)
        else:
            return self.read_byte_block(i2c_addr, register, length)
        
    def _set_regs(self, i2c_addr, register, value):
        """Set one or more registers."""
        if isinstance(value, int):
            self.write_byte(i2c_addr, register, value)
        else:
            self.write_byte_block(i2c_addr, register, value)

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

    COTR_DEF_STATE       =  0x55
    COTR_POS_STATE       =  0xAA

    # Customization registers
    KX13X_COTR             = 0x12
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
    KX13X_XOUT_H           = 0x09
    KX13X_YOUT_L           = 0x0A
    KX13X_YOUT_H           = 0x0B
    KX13X_ZOUT_L           = 0x0C
    KX13X_ZOUT_H           = 0x0D


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

    def run_command_test(self):
        '''Does self test on accelerometer ensuring it is working properly'''
        reg_val = self.i2c.read_byte(self.addr, self.KX13X_CNTL2)
        reg_val &= 0xBF
        reg_val |= (1 << 6)
        self.i2c.write_byte(self.addr, self.KX13X_CNTL2 , reg_val)

        reg_val = self.i2c.read_byte(self.addr, self.KX13X_COTR)
        if reg_val == self.COTR_POS_STATE:
            return True
        else:
            return False

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

    def get_g_range(self):
        reg_val = self.i2c.read_byte(self.addr, self.KX13X_CNTL1)
        reg_val &= 0x18
        reg_val = reg_val >> 3

        if reg_val == 0x00:
            return 2
        elif reg_val == 0x01:
            return 4
        elif reg_val == 0x02:
            return 8
        elif reg_val == 0x03:
            return 16

    def get_raw_accel_values(self):

        reg_val = self.i2c.read_byte(self.addr, self.KX13X_INC4) #These next two lines sketch me out
        if reg_val & 0x40:
            accel_data = self.i2c.read_byte_block(self.addr, self.KX13X_XOUT_L, self.TOTAL_ACCEL_DATA_16BIT)
            xData = int.from_bytes(accel_data[0:2], byteorder='little', signed=True)
            yData =  int.from_bytes(accel_data[2:4], byteorder='little', signed=True)
            zData =  int.from_bytes(accel_data[4:6], byteorder='little', signed=True)
        else:
            accel_data = self.i2c.read_byte_block(self.addr, self.KX13X_XOUT_L, self.TOTAL_ACCEL_DATA_8BIT)
            xData = int.from_bytes(self.i2c.read_byte(self.addr, self.KX13X_XOUT_H), signed=True)
            yData = int.from_bytes(self.i2c.read_byte(self.addr, self.KX13X_XOUT_H), signed=True)
            zData = int.from_bytes(self.i2c.read_byte(self.addr, self.KX13X_XOUT_H), signed=True)

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


##BME680 Temperature, Pressure, Humidity & Gas Sensor. 
##Sensory library adapted for our I2C protocol from pimoroni: https://github.com/pimoroni/bme680-python 
## In addition we added a couple functions for our dedicated need such as the get altitude function

BME680_I2C_ADDRESS = 0x76

__version__ = '1.1.1'


# Export constants to global namespace
# so end-users can "from BME680 import NAME"
if hasattr(BME680_constants, '__dict__'):
    for key in BME680_constants.__dict__:
        value = BME680_constants.__dict__[key]
        if key not in globals():
            globals()[key] = value


class BME680(BME680Data):
    """BOSCH BME680.

    Gas, pressure, temperature and humidity sensor.

    :param i2c_addr: One of I2C_ADDR_PRIMARY (0x76) or I2C_ADDR_SECONDARY (0x77)
    :param i2c_device: Optional smbus or compatible instance for facilitating i2c communications.

    """

    def __init__(self, bus=None, network=None, sea_level_pressure=1000, addr=BME680_I2C_ADDRESS):
        """Initialise BME680 sensor instance and verify device presence.
        """
        if network == None:
            self.i2c = create_unified_i2c(bus=bus)
        else:
            self.i2c = network
        self.addr = addr

        # Added stuff to make it actually work on PI and get altitude
        self.sea_level_pressure = sea_level_pressure
        self.data = BME680_constants.FieldData()
        self.calibration_data = BME680_constants.CalibrationData()
        self.tph_settings = BME680_constants.TPHSettings()
        self.gas_settings = BME680_constants.GasSettings()

        self._variant = self.i2c._get_regs(self.addr, BME680_constants.CHIP_VARIANT_ADDR, 1)

        self.soft_reset()
        self.set_power_mode(BME680_constants.SLEEP_MODE)

        self._get_calibration_data()

        self.set_humidity_oversample(BME680_constants.OS_2X)
        self.set_pressure_oversample(BME680_constants.OS_4X)
        self.set_temperature_oversample(BME680_constants.OS_8X)
        self.set_filter(BME680_constants.FILTER_SIZE_3)
        if self._variant == BME680_constants.VARIANT_HIGH:
            self.set_gas_status(BME680_constants.ENABLE_GAS_MEAS_HIGH)
        else:
            self.set_gas_status(BME680_constants.ENABLE_GAS_MEAS_LOW)
        self.set_temp_offset(0)
        self.get_sensor_data()

    def _get_calibration_data(self):
        """Retrieve the sensor calibration data and store it in .calibration_data."""
        calibration = self.i2c._get_regs(self.addr, BME680_constants.COEFF_ADDR1, BME680_constants.COEFF_ADDR1_LEN)
        calibration += self.i2c._get_regs(self.addr, BME680_constants.COEFF_ADDR2, BME680_constants.COEFF_ADDR2_LEN)

        heat_range = self.i2c._get_regs(self.addr, BME680_constants.ADDR_RES_HEAT_RANGE_ADDR, 1)
        heat_value = BME680_constants.twos_comp(self.i2c._get_regs(self.addr, BME680_constants.ADDR_RES_HEAT_VAL_ADDR, 1), bits=8)
        sw_error = BME680_constants.twos_comp(self.i2c._get_regs(self.addr, BME680_constants.ADDR_RANGE_SW_ERR_ADDR, 1), bits=8)

        self.calibration_data.set_from_array(calibration)
        self.calibration_data.set_other(heat_range, heat_value, sw_error)

    def soft_reset(self):
        """Trigger a soft reset."""
        self.i2c._set_regs(self.addr, BME680_constants.SOFT_RESET_ADDR, BME680_constants.SOFT_RESET_CMD)
        time.sleep(BME680_constants.RESET_PERIOD / 1000.0)

    def set_temp_offset(self, value):
        """Set temperature offset in celsius.

        If set, the temperature t_fine will be increased by given value in celsius.
        :param value: Temperature offset in Celsius, eg. 4, -8, 1.25

        """
        if value == 0:
            self.offset_temp_in_t_fine = 0
        else:
            self.offset_temp_in_t_fine = int(math.copysign((((int(abs(value) * 100)) << 8) - 128) / 5, value))

    def set_humidity_oversample(self, value):
        """Set humidity oversampling.

        A higher oversampling value means more stable sensor readings,
        with less noise and jitter.

        However each step of oversampling adds about 2ms to the latency,
        causing a slower response time to fast transients.

        :param value: Oversampling value, one of: OS_NONE, OS_1X, OS_2X, OS_4X, OS_8X, OS_16X

        """
        self.tph_settings.os_hum = value
        self._set_bits(BME680_constants.CONF_OS_H_ADDR, BME680_constants.OSH_MSK, BME680_constants.OSH_POS, value)

    def get_humidity_oversample(self):
        """Get humidity oversampling."""
        return (self.i2c._get_regs(self.addr, BME680_constants.CONF_OS_H_ADDR, 1) & BME680_constants.OSH_MSK) >> BME680_constants.OSH_POS

    def set_pressure_oversample(self, value):
        """Set temperature oversampling.

        A higher oversampling value means more stable sensor readings,
        with less noise and jitter.

        However each step of oversampling adds about 2ms to the latency,
        causing a slower response time to fast transients.

        :param value: Oversampling value, one of: OS_NONE, OS_1X, OS_2X, OS_4X, OS_8X, OS_16X

        """
        self.tph_settings.os_pres = value
        self._set_bits(BME680_constants.CONF_T_P_MODE_ADDR, BME680_constants.OSP_MSK, BME680_constants.OSP_POS, value)

    def get_pressure_oversample(self):
        """Get pressure oversampling."""
        return (self.i2c._get_regs(self.addr, BME680_constants.CONF_T_P_MODE_ADDR, 1) & BME680_constants.OSP_MSK) >> BME680_constants.OSP_POS

    def set_temperature_oversample(self, value):
        """Set pressure oversampling.

        A higher oversampling value means more stable sensor readings,
        with less noise and jitter.

        However each step of oversampling adds about 2ms to the latency,
        causing a slower response time to fast transients.

        :param value: Oversampling value, one of: OS_NONE, OS_1X, OS_2X, OS_4X, OS_8X, OS_16X

        """
        self.tph_settings.os_temp = value
        self._set_bits(BME680_constants.CONF_T_P_MODE_ADDR, BME680_constants.OST_MSK, BME680_constants.OST_POS, value)

    def get_temperature_oversample(self):
        """Get temperature oversampling."""
        return (self.i2c._get_regs(self.addr, BME680_constants.CONF_T_P_MODE_ADDR, 1) & BME680_constants.OST_MSK) >> BME680_constants.OST_POS

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
        self._set_bits(BME680_constants.CONF_ODR_FILT_ADDR, BME680_constants.FILTER_MSK, BME680_constants.FILTER_POS, value)

    def get_filter(self):
        """Get filter size."""
        return (self.i2c._get_regs(self.addr, BME680_constants.CONF_ODR_FILT_ADDR, 1) & BME680_constants.FILTER_MSK) >> BME680_constants.FILTER_POS

    def select_gas_heater_profile(self, value):
        """Set current gas sensor conversion profile.

        Select one of the 10 configured heating durations/set points.

        :param value: Profile index from 0 to 9

        """
        if value > BME680_constants.NBCONV_MAX or value < BME680_constants.NBCONV_MIN:
            raise ValueError("Profile '{}' should be between {} and {}".format(value, BME680_constants.NBCONV_MIN, BME680_constants.NBCONV_MAX))

        self.gas_settings.nb_conv = value
        self._set_bits(BME680_constants.CONF_ODR_RUN_GAS_NBC_ADDR, BME680_constants.NBCONV_MSK, BME680_constants.NBCONV_POS, value)

    def get_gas_heater_profile(self):
        """Get gas sensor conversion profile: 0 to 9."""
        return self.i2c._get_regs(self.addr, BME680_constants.CONF_ODR_RUN_GAS_NBC_ADDR, 1) & BME680_constants.NBCONV_MSK

    def set_gas_heater_status(self, value):
        """Enable/disable gas heater."""
        self.gas_settings.heater = value
        self._set_bits(BME680_constants.CONF_HEAT_CTRL_ADDR, BME680_constants.HCTRL_MSK, BME680_constants.HCTRL_POS, value)

    def get_gas_heater_status(self):
        """Get current heater status."""
        return (self.i2c._get_regs(self.addr, BME680_constants.CONF_HEAT_CTRL_ADDR, 1) & BME680_constants.HCTRL_MSK) >> BME680_constants.HCTRL_POS

    def set_gas_status(self, value):
        """Enable/disable gas sensor."""
        if value == -1:
            if self._variant == BME680_constants.VARIANT_HIGH:
                value = BME680_constants.ENABLE_GAS_MEAS_HIGH
            else:
                value = BME680_constants.ENABLE_GAS_MEAS_LOW
        self.gas_settings.run_gas = value
        self._set_bits(BME680_constants.CONF_ODR_RUN_GAS_NBC_ADDR, BME680_constants.RUN_GAS_MSK, BME680_constants.RUN_GAS_POS, value)

    def get_gas_status(self):
        """Get the current gas status."""
        return (self.i2c._get_regs(self.addr, BME680_constants.CONF_ODR_RUN_GAS_NBC_ADDR, 1) & BME680_constants.RUN_GAS_MSK) >> BME680_constants.RUN_GAS_POS

    def set_gas_heater_profile(self, temperature, duration, nb_profile=0):
        """Set temperature and duration of gas sensor heater.

        :param temperature: Target temperature in degrees celsius, between 200 and 400
        :param durarion: Target duration in milliseconds, between 1 and 4032
        :param nb_profile: Target profile, between 0 and 9

        """
        self.set_gas_heater_temperature(temperature, nb_profile=nb_profile)
        self.set_gas_heater_duration(duration, nb_profile=nb_profile)

    def set_gas_heater_temperature(self, value, nb_profile=0):
        """Set gas sensor heater temperature.

        :param value: Target temperature in degrees celsius, between 200 and 400

        When setting an nb_profile other than 0,
        make sure to select it with select_gas_heater_profile.

        """
        if nb_profile > BME680_constants.NBCONV_MAX or value < BME680_constants.NBCONV_MIN:
            raise ValueError('Profile "{}" should be between {} and {}'.format(nb_profile, BME680_constants.NBCONV_MIN, BME680_constants.NBCONV_MAX))

        self.gas_settings.heatr_temp = value
        temp = int(self._calc_heater_resistance(self.gas_settings.heatr_temp))
        self.i2c._set_regs(self.addr, BME680_constants.RES_HEAT0_ADDR + nb_profile, temp)

    def set_gas_heater_duration(self, value, nb_profile=0):
        """Set gas sensor heater duration.

        Heating durations between 1 ms and 4032 ms can be configured.
        Approximately 20-30 ms are necessary for the heater to reach the intended target temperature.

        :param value: Heating duration in milliseconds.

        When setting an nb_profile other than 0,
        make sure to select it with select_gas_heater_profile.

        """
        if nb_profile > BME680_constants.NBCONV_MAX or value < BME680_constants.NBCONV_MIN:
            raise ValueError('Profile "{}" should be between {} and {}'.format(nb_profile, BME680_constants.NBCONV_MIN, BME680_constants.NBCONV_MAX))

        self.gas_settings.heatr_dur = value
        temp = self._calc_heater_duration(self.gas_settings.heatr_dur)
        self.i2c._set_regs(self.addr, BME680_constants.GAS_WAIT0_ADDR + nb_profile, temp)

    def set_power_mode(self, value, blocking=True):
        """Set power mode."""
        if value not in (BME680_constants.SLEEP_MODE, BME680_constants.FORCED_MODE):
            raise ValueError('Power mode should be one of SLEEP_MODE or FORCED_MODE')

        self.power_mode = value

        self._set_bits(BME680_constants.CONF_T_P_MODE_ADDR, BME680_constants.MODE_MSK, BME680_constants.MODE_POS, value)

        while blocking and self.get_power_mode() != self.power_mode:
            time.sleep(BME680_constants.POLL_PERIOD_MS / 1000.0)

    def get_power_mode(self):
        """Get power mode."""
        self.power_mode = self.i2c._get_regs(self.addr, BME680_constants.CONF_T_P_MODE_ADDR, 1)
        return self.power_mode

    def get_sensor_data(self):
        """Get sensor data.

        Stores data in .data and returns True upon success.

        """
        self.set_power_mode(BME680_constants.FORCED_MODE)

        for attempt in range(10):
            status = self.i2c._get_regs(self.addr, BME680_constants.FIELD0_ADDR, 1)

            if (status & BME680_constants.NEW_DATA_MSK) == 0:
                time.sleep(BME680_constants.POLL_PERIOD_MS / 1000.0)
                continue

            regs = self.i2c._get_regs(self.addr, BME680_constants.FIELD0_ADDR, BME680_constants.FIELD_LENGTH)

            self.data.status = regs[0] & BME680_constants.NEW_DATA_MSK
            # Contains the nb_profile used to obtain the current measurement
            self.data.gas_index = regs[0] & BME680_constants.GAS_INDEX_MSK
            self.data.meas_index = regs[1]

            adc_pres = (regs[2] << 12) | (regs[3] << 4) | (regs[4] >> 4)
            adc_temp = (regs[5] << 12) | (regs[6] << 4) | (regs[7] >> 4)
            adc_hum = (regs[8] << 8) | regs[9]
            adc_gas_res_low = (regs[13] << 2) | (regs[14] >> 6)
            adc_gas_res_high = (regs[15] << 2) | (regs[16] >> 6)
            gas_range_l = regs[14] & BME680_constants.GAS_RANGE_MSK
            gas_range_h = regs[16] & BME680_constants.GAS_RANGE_MSK

            if self._variant == BME680_constants.VARIANT_HIGH:
                self.data.status |= regs[16] & BME680_constants.GASM_VALID_MSK
                self.data.status |= regs[16] & BME680_constants.HEAT_STAB_MSK
            else:
                self.data.status |= regs[14] & BME680_constants.GASM_VALID_MSK
                self.data.status |= regs[14] & BME680_constants.HEAT_STAB_MSK

            self.data.heat_stable = (self.data.status & BME680_constants.HEAT_STAB_MSK) > 0

            temperature = self._calc_temperature(adc_temp)
            self.data.temperature = temperature / 100.0
            self.ambient_temperature = temperature  # Saved for heater calc

            self.data.pressure = self._calc_pressure(adc_pres) / 100.0
            self.data.humidity = self._calc_humidity(adc_hum) / 1000.0

            if self._variant == BME680_constants.VARIANT_HIGH:
                self.data.gas_resistance = self._calc_gas_resistance_high(adc_gas_res_high, gas_range_h)
            else:
                self.data.gas_resistance = self._calc_gas_resistance_low(adc_gas_res_low, gas_range_l)

            return True

        return False


    def _calc_temperature(self, temperature_adc):
        """Convert the raw temperature to degrees C using calibration_data."""
        var1 = (temperature_adc >> 3) - (self.calibration_data.par_t1 << 1)
        var2 = (var1 * self.calibration_data.par_t2) >> 11
        var3 = ((var1 >> 1) * (var1 >> 1)) >> 12
        var3 = ((var3) * (self.calibration_data.par_t3 << 4)) >> 14

        # Save teperature data for pressure calculations
        self.calibration_data.t_fine = (var2 + var3) + self.offset_temp_in_t_fine
        calc_temp = (((self.calibration_data.t_fine * 5) + 128) >> 8)

        return calc_temp

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

    def _calc_humidity(self, humidity_adc):
        """Convert the raw humidity using calibration data."""
        temp_scaled = ((self.calibration_data.t_fine * 5) + 128) >> 8
        var1 = (humidity_adc - ((self.calibration_data.par_h1 * 16))) -\
               (((temp_scaled * self.calibration_data.par_h3) // (100)) >> 1)
        var2 = (self.calibration_data.par_h2 *
                (((temp_scaled * self.calibration_data.par_h4) // (100)) +
                 (((temp_scaled * ((temp_scaled * self.calibration_data.par_h5) // (100))) >> 6) //
                 (100)) + (1 * 16384))) >> 10
        var3 = var1 * var2
        var4 = self.calibration_data.par_h6 << 7
        var4 = ((var4) + ((temp_scaled * self.calibration_data.par_h7) // (100))) >> 4
        var5 = ((var3 >> 14) * (var3 >> 14)) >> 10
        var6 = (var4 * var5) >> 1
        calc_hum = (((var3 + var6) >> 10) * (1000)) >> 12

        return min(max(calc_hum, 0), 100000)

    def _calc_gas_resistance(self, gas_res_adc, gas_range):
        """Convert the raw gas resistance using calibration data."""
        if self._variant == BME680_constants.VARIANT_HIGH:
            return self._calc_gas_resistance_high(gas_res_adc, gas_range)
        else:
            return self._calc_gas_resistance_low(gas_res_adc, gas_range)

    def _calc_gas_resistance_high(self, gas_res_adc, gas_range):
        """Convert the raw gas resistance using calibration data.

        Applies to Variant ID == 0x01 only.

        """
        var1 = 262144 >> gas_range
        var2 = gas_res_adc - 512

        var2 *= 3
        var2 = 4096 + var2

        calc_gas_res = (10000 * var1) / var2
        calc_gas_res *= 100

        return calc_gas_res

    def _calc_gas_resistance_low(self, gas_res_adc, gas_range):
        """Convert the raw gas resistance using calibration data.

        Applies to Variant ID == 0x00 only.

        """
        var1 = ((1340 + (5 * self.calibration_data.range_sw_err)) * (lookupTable1[gas_range])) >> 16
        var2 = (((gas_res_adc << 15) - (16777216)) + var1)
        var3 = ((lookupTable2[gas_range] * var1) >> 9)
        calc_gas_res = ((var3 + (var2 >> 1)) / var2)

        if calc_gas_res < 0:
            calc_gas_res = (1 << 32) + calc_gas_res

        return calc_gas_res

    def _calc_heater_resistance(self, temperature):
        """Convert raw heater resistance using calibration data."""
        temperature = min(max(temperature, 200), 400)

        var1 = ((self.ambient_temperature * self.calibration_data.par_gh3) / 1000) * 256
        var2 = (self.calibration_data.par_gh1 + 784) * (((((self.calibration_data.par_gh2 + 154009) * temperature * 5) / 100) + 3276800) / 10)
        var3 = var1 + (var2 / 2)
        var4 = (var3 / (self.calibration_data.res_heat_range + 4))
        var5 = (131 * self.calibration_data.res_heat_val) + 65536
        heatr_res_x100 = (((var4 / var5) - 250) * 34)
        heatr_res = ((heatr_res_x100 + 50) / 100)

        return heatr_res

    def _calc_heater_duration(self, duration):
        """Calculate correct value for heater duration setting from milliseconds."""
        if duration < 0xfc0:
            factor = 0

            while duration > 0x3f:
                duration /= 4
                factor += 1

            return int(duration + (factor * 64))

        return 0xff
    
    def _set_bits(self, register, mask, position, value):
        """Mask out and set one or more bits in a register."""
        temp = self.i2c._get_regs(self.addr, register, 1)
        temp &= ~mask
        temp |= value << position
        self.i2c._set_regs(self.addr, register, temp)

    def get_altitude(self):
        if self.get_sensor_data():
            alt = 44307.69396 * (1 - (( self.data.pressure / self.sea_level_pressure) ** 0.190284))
            self.altitude = alt
            return alt
        else:
            return False

##############################################################################################################################################
##############################################################################################################################################

## This code is inspired by the https://www.waveshare.com/wiki/Pico-GPS-L76B code module
## We have adapted the code from this library for the raspberry pi 4. 

UART_PIN = '/dev/ttyS0'
Temp = '0123456789ABCDEF*'
a = 6378245.0
ee = 0.00669342162296594323
x_pi = math.pi * 3000.0 / 180.0

class L76B(object):

    Lon = 0.0
    Lat = 0.0
    Lon_area = 'E'
    Lat_area = 'W'
    Time_H = 0
    Time_M = 0
    Time_S = 0
    Status = 0
    Lon_Baidu = 0.0
    Lat_Baidu = 0.0
    Lon_Google = 0.0
    Lat_Google = 0.0

    SET_NMEA_OUTPUT = '$PMTK314,1,1,1,1,1,1,1,1,0,0,0,0,0,0,0,0,0,1,0'

    #Baud rate
    SET_NMEA_BAUDRATE          = '$PMTK251'
    SET_NMEA_BAUDRATE_115200   = 115200
    SET_NMEA_BAUDRATE_57600    = 57600
    SET_NMEA_BAUDRATE_38400    = 38400
    SET_NMEA_BAUDRATE_19200    = 19200
    SET_NMEA_BAUDRATE_9600     = 9600
    SET_NMEA_BAUDRATE_4800     = 4800

    #Switching time output
    SET_SYNC_PPS_NMEA_OFF   = '$PMTK255,0'
    SET_SYNC_PPS_NMEA_ON    = '$PMTK255,1'

    def __init__(self, StandByPin, ForcePin):
        self.ser = serial.Serial(port=UART_PIN, 
                                 baudrate=9600, 
                                 parity=serial.PARITY_NONE, 
                                 stopbits=serial.STOPBITS_ONE,
                                 bytesize=serial.EIGHTBITS,
                                 timeout=1000)
        self.Force = GPIO.OutputDevice(StandByPin, active_high=True, initial_value=False)
        self.Standby = GPIO.OutputDevice(ForcePin, active_high=True, initial_value=False)

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
        print(data)   

    def l76x_exit_backup_mode(self):
        self.Force.on()
        time.sleep(1)
        self.Force.off()
        time.sleep(1)

    def gcj02_to_bd09(self,lng, lat):
        z = math.sqrt(lng * lng + lat * lat) + 0.00002 * math.sin(lat * x_pi)
        theta = math.atan2(lat, lng) + 0.000003 * math.cos(lng * x_pi)
        bd_lng = z * math.cos(theta) + 0.0065
        bd_lat = z * math.sin(theta) + 0.006
        return [bd_lng, bd_lat]

    def bd09_to_gcj02(self,bd_lon, bd_lat):
        x = bd_lon - 0.0065
        y = bd_lat - 0.006
        z = math.sqrt(x * x + y * y) - 0.00002 * math.sin(y * x_pi)
        theta = math.atan2(y, x) - 0.000003 * math.cos(x * x_pi)
        gg_lng = z * math.cos(theta)
        gg_lat = z * math.sin(theta)
        return [gg_lng, gg_lat]

    def wgs84_to_gcj02(self,lng, lat):
        if self.out_of_china(lng, lat): 
            return [lng, lat]
        dlat = self._transformlat(lng - 105.0, lat - 35.0)
        dlng = self._transformlng(lng - 105.0, lat - 35.0)
        radlat = lat / 180.0 * math.pi
        magic = math.sin(radlat)
        magic = 1 - ee * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
        dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
        mglat = lat + dlat
        mglng = lng + dlng
        return [mglng, mglat]


    def gcj02_to_wgs84(self,lng, lat):
        if self.out_of_china(lng, lat):
            return [lng, lat]
        dlat = self._transformlat(lng - 105.0, lat - 35.0)
        dlng = self._transformlng(lng - 105.0, lat - 35.0)
        radlat = lat / 180.0 * math.pi
        magic = math.sin(radlat)
        magic = 1 - ee * magic * magic
        sqrtmagic = math.sqrt(magic)
        dlat = (dlat * 180.0) / ((a * (1 - ee)) / (magic * sqrtmagic) * math.pi)
        dlng = (dlng * 180.0) / (a / sqrtmagic * math.cos(radlat) * math.pi)
        mglat = lat + dlat
        mglng = lng + dlng
        return [lng * 2 - mglng, lat * 2 - mglat]

    def bd09_to_wgs84(self,bd_lon, bd_lat):
        lon, lat = self.bd09_to_gcj02(bd_lon, bd_lat)
        return self.gcj02_to_wgs84(lon, lat)

    def wgs84_to_bd09(self,lon, lat):
        lon, lat = self.wgs84_to_gcj02(lon, lat)
#         return gcj02_to_bd09(lon, lat)
        self.Lon_Baidu,self.Lat_Baidu = self.gcj02_to_bd09(lon, lat)
    
    def out_of_china(self,lng, lat):
        return not (lng > 73.66 and lng < 135.05 and lat > 3.86 and lat < 53.55)

    def _transformlat(self,lng, lat):
        ret = -100.0 + 2.0 * lng + 3.0 * lat + 0.2 * lat * lat + \
            0.1 * lng * lat + 0.2 * math.sqrt(math.fabs(lng))
        ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 *
                math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lat * math.pi) + 40.0 *
                math.sin(lat / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (160.0 * math.sin(lat / 12.0 * math.pi) + 320 *
                math.sin(lat * math.pi / 30.0)) * 2.0 / 3.0
        return ret


    def _transformlng(self,lng, lat):
        ret = 300.0 + lng + 2.0 * lat + 0.1 * lng * lng + \
            0.1 * lng * lat + 0.1 * math.sqrt(math.fabs(lng))
        ret += (20.0 * math.sin(6.0 * lng * math.pi) + 20.0 *
                math.sin(2.0 * lng * math.pi)) * 2.0 / 3.0
        ret += (20.0 * math.sin(lng * math.pi) + 40.0 *
                math.sin(lng / 3.0 * math.pi)) * 2.0 / 3.0
        ret += (150.0 * math.sin(lng / 12.0 * math.pi) + 300.0 *
                math.sin(lng / 30.0 * math.pi)) * 2.0 / 3.0
        return ret

    def l76x_set_baudrate(self, _baudrate):
        self.ser.baudrate = _baudrate

    def uart_send_byte(self, value):
        self.ser.write(value) 

    def uart_send_string(self, value): 
        self.ser.write(value)

    def uart_receive_byte(self): 
        return self.ser.read(1)

    def uart_receiveString(self, value): 
        data = self.ser.read(value)
        return data
    
################################################################################################################################
################################################################################################################################
    
## MOTOR CONTROL CLASSES

class Motor(object):  ### Instatiate with this in name    pi = pigpio.pi() 

    def __init__(self, pi, PWM_pin): #Pin is 0-31
        self.pin = PWM_pin
        self.pi = pi

    def set_speed(self, speed): #takes in motor object and speed to set it to. speed 0-255
        self.pi.set_PWM_dutycycle(self.pin, speed) # PWM full 

####################################################################################################################################
####################################################################################################################################
    
## Raspberry Pi Noir CLASSES
## Must change to Picamera 2
class PiNoir(object):

    def __init__(self, photo_filename):
        self.camera = Picamera2()
        self.filename = photo_filename

    def capture_image(self, format = None, size = None):
        try:
            self.camera.start_preview()
            time.sleep(2)
            self.camera.capture(self.filename, format=format, resize=size)
        except Exception: 
            return False
        return True

    def continous_caputre(self, format = None, size = None, num_photos = 2, delay = 1):
        try:
            self.camera.start_preview()
            time.sleep(2)
            for i in range(num_photos):
                self.camera.capture_continuous(self.filename, format=format, resize=size)
                #Should do somehting here wiht photo before it is replaced
                time.sleep(delay)
        except Exception:
            return False
        return True
            
