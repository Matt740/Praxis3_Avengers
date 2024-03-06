from machine import I2C, Pin
from utime import sleep_ms
from math import sqrt, atan2

# I2C BASES
###################################################################################################################################################

i2c_err_str = 'Pico board could not communicate with module at address 0x{:02X}, check wiring'
compat_ind = 1

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
            self.i2c = I2C(0, scl=Pin(9), sda=Pin(8), freq=freq) # RPi Pico in Expansion Board
        else:
            raise Exception("Please provide at least bus, sda, and scl")

        self.writeto_mem = self.i2c.writeto_mem
        self.readfrom_mem = self.i2c.readfrom_mem

    def write8(self, addr, reg, data):
        if reg is None:
            self.i2c.writeto(addr, data)
        else:
            self.i2c.writeto(addr, reg + data)
            
    def read16(self, addr, reg):
        self.i2c.writeto(addr, reg, False)
        return self.i2c.readfrom(addr, 2)
        
    def scan(self):
        print([hex(i) for i in self.i2c.scan()])

def create_unified_i2c(bus=None, freq=None, sda=None, scl=None, suppress_warnings=True):
    i2c = I2CUnifiedMachine(bus=bus, freq=freq, sda=sda, scl=scl)
    return i2c

###################################################################################################################################################
    
#BMP280 Code Base
#################################################################################################################

#Sensor ID address
_BMP280_ADDRESS = 0x76

#Over Sampling Codes
SKIPPED = 0b000
#Pressure
_SAMP_PRES_X1 = 0b001
_SAMP_PRES_X2 = 0b010
_SAMP_PRES_X4 = 0b011
_SAMP_PRES_X8 = 0b100
_SAMP_PRES_X16 = 0b111

#Temperature
_SAMP_TEMP_X1 = 0b001
_SAMP_TEMP_X2 = 0b010
_SAMP_TEMP_X4 = 0b011
_SAMP_TEMP_X8 = 0b100
_SAMP_TEMP_X16 = 0b111

_OVER_SAMP = {0: 0b000, 1: 0b001, 2: 0b010, 4: 0b011, 8: 0b100, 16: 0b111}

#Power Modes
_SLEEP_MODE = 0b00
_FORCED_MODE = 0b10
_NORMAL_MODE = 0b11

_MODE = {"SLEEP": 0b00, "FORCED": 0b10, "NORMAL": 0b11}

#Standby Time
_STANDBY_0_5 = 0b000
_STANBY_62_5 = 0b001
_STANDBY_125 = 0b010
_STANDBY_250 = 0b011
_STANDBY_500 = 0b100
_STANDBY_1000 = 0b101
_STANDBY_2000 = 0b110
_STANDBY_4000 = 0b111

_STANDBY = {0.5: 0b000, 62.5: 0b001, 125: 0b010, 250: 0b011, 500: 0b100, 1000: 0b101, 2000: 0b110, 4000: 0b111}

class BMP280(object):
    def __init__(self, bus=None, freq=None, sda=None, scl=None, network=None, sea_level_pressure=None, addr=_BMP280_ADDRESS):
        if network == None:
            self.i2c = create_unified_i2c(bus=bus, freq=freq, sda=sda, scl=scl)
        else:
            self.i2c = network
        self.addr = addr
        try:
            #wake up device out of sleep mode
            self.i2c.writeto_mem(self.addr, 0xF4, bytes([0x27]))
            sleep_ms(5)
        except Exception as e:
            print(i2c_err_str.format(self.addr))
            raise e
        self.get_calibration()
        self.t_fine = 0
        self.t_standby = self.get_t_standby()
        self.over_press = self.get_over_press()
        self.over_temp = self.get_over_temp()
        self.mode = self.get_mode()
        if sea_level_pressure != None:
            self.p0 = sea_level_pressure
        else:
            self.p0 = 1013.25

    def get_t_standby(self):
        read_byte = self.i2c.readfrom_mem(self.addr, 0xF5, 1)
        wanted_bits = int.from_bytes(read_byte, 'little') & 0xE0
        if wanted_bits == 0x00:
            return 0.5
        elif wanted_bits == 0x20:
            return 62.5
        elif wanted_bits == 0x40:
            return 125
        elif wanted_bits == 0x60:
            return 250
        elif wanted_bits == 0x80:
            return 500
        elif wanted_bits == 0xA0:
            return 1000  
        elif wanted_bits == 0xC0:
            return 2000  
        elif wanted_bits == 0xE0:
            return 4000
        else:
            return None
        
    def get_over_press(self):
        read_byte = self.i2c.readfrom_mem(self.addr, 0xF4, 1)
        wanted_bits = int.from_bytes(read_byte, 'little') & 0x1C
        if wanted_bits == 0x00:
            return 0
        elif wanted_bits == 0x04:
            return 1
        elif wanted_bits == 0x08:
            return 2
        elif wanted_bits == 0x0C:
            return 4
        elif wanted_bits == 0x10:
            return 8
        elif wanted_bits == 0x14:
            return 16
        elif wanted_bits == 0x18:
            return 16
        elif wanted_bits == 0x1C:
            return 16
        else:
            return None
    
    def get_over_temp(self):
        read_byte = self.i2c.readfrom_mem(self.addr, 0xF4, 1)
        wanted_bits = int.from_bytes(read_byte, 'little') & 0xE0
        if wanted_bits == 0x00:
            return 0
        elif wanted_bits == 0x20:
            return 1
        elif wanted_bits == 0x40:
            return 2
        elif wanted_bits == 0x60:
            return 4
        elif wanted_bits == 0x80:
            return 8
        elif wanted_bits == 0xA0:
            return 16 
        elif wanted_bits == 0xC0:
            return 16
        elif wanted_bits == 0xE0:
            return 16
        else:
            return None
    
    def get_mode(self):
        read_byte = self.i2c.readfrom_mem(self.addr, 0xF4, 1)
        wanted_bits = int.from_bytes(read_byte, 'little') & 0x03
        if wanted_bits == 0x00:
            return "SLEEP"
        elif wanted_bits == 0x01:
            return "FORCED"
        elif wanted_bits == 0x02:
            return "FORCED"
        elif wanted_bits == 0x03:
            return "NORMAL"
        else:
            return None
    
    def set_ctrl_meas(self, over_temp=0, over_press=0, mode="SLEEP"):
        ctrl_meas = (_OVER_SAMP[over_temp] << 5) | ((_OVER_SAMP[over_press] << 5) >> 3) | ((_MODE[mode] << 6) >> 6)
        self.i2c.writeto_mem(self.addr, 0xF4, bytes([ctrl_meas]))
        self.over_press = self.get_over_press()
        self.over_temp = self.get_over_temp()
        self.mode = self.get_mode()
    
    def set_config(self, standby=0.5):
        cur_config = self.i2c.readfrom_mem(self.addr, 0xF4, 1)
        cur_filter = int.from_bytes(cur_config, 'little') & 0x1F
        config = (_STANDBY[standby] << 5) | cur_filter
        self.i2c.writeto_mem(self.addr, 0xF5, bytes([config]))
        self.t_standby = self.get_t_standby()

    def get_calibration(self):
        self.dig_T1 = int.from_bytes(self.i2c.readfrom_mem(self.addr, 0x88, 2), 'little')
        self.dig_T2 = signedIntFromBytes(self.i2c.readfrom_mem(self.addr, 0x8A, 2), 'little')
        self.dig_T3 = signedIntFromBytes(self.i2c.readfrom_mem(self.addr, 0x8C, 2), 'little')

        self.dig_P1 = int.from_bytes(self.i2c.readfrom_mem(self.addr, 0x8E, 2), 'little')
        self.dig_P2 = signedIntFromBytes(self.i2c.readfrom_mem(self.addr, 0x90, 2), 'little')
        self.dig_P3 = signedIntFromBytes(self.i2c.readfrom_mem(self.addr, 0x92, 2), 'little')
        self.dig_P4 = signedIntFromBytes(self.i2c.readfrom_mem(self.addr, 0x94, 2), 'little')
        self.dig_P5 = signedIntFromBytes(self.i2c.readfrom_mem(self.addr, 0x96, 2), 'little')
        self.dig_P6 = signedIntFromBytes(self.i2c.readfrom_mem(self.addr, 0x98, 2), 'little')
        self.dig_P7 = signedIntFromBytes(self.i2c.readfrom_mem(self.addr, 0x9A, 2), 'little')
        self.dig_P8 = signedIntFromBytes(self.i2c.readfrom_mem(self.addr, 0x9C, 2), 'little')
        self.dig_P9 = signedIntFromBytes(self.i2c.readfrom_mem(self.addr, 0x9E, 2), 'little')

    def read_raw_pressure(self):
        p_msb = self.i2c.readfrom_mem(self.addr, 0xF7, 1)
        p_lsb = self.i2c.readfrom_mem(self.addr, 0xF8, 1)
        p_xlsb = self.i2c.readfrom_mem(self.addr, 0xF9, 1)
        raw_pressure = ((int.from_bytes(p_msb, "little") << 16) | (int.from_bytes(p_lsb, "little") << 8) | int.from_bytes(p_xlsb, "little")) >> 4

        return raw_pressure
    
    def read_raw_temperature(self):
        t_msb = self.i2c.readfrom_mem(self.addr, 0xFA, 1)
        t_lsb = self.i2c.readfrom_mem(self.addr, 0xFB, 1)
        t_xlsb = self.i2c.readfrom_mem(self.addr, 0xFC, 1)
        raw_temperature = ((int.from_bytes(t_msb, "little") << 16) | (int.from_bytes(t_lsb, "little") << 8) | int.from_bytes(t_xlsb, "little")) >> 4
        
        return raw_temperature
    
    def temperature(self):
        # Temp in degree celcius
        raw = self.read_raw_temperature()
        var1 = ((raw >> 3) - (self.dig_T1 << 1)) * (self.dig_T2 >> 11)
        var2 = (((((raw >> 4) - self.dig_T1) * ((raw >> 4) - self.dig_T1)) >> 12) * self.dig_T3) >> 14
        self.t_fine = var1 + var2

        t = (self.t_fine * 5 + 128) >> 8
        temp = t / 100
        return temp

    def pressure(self):
        # Presure in hPa
        raw = self.read_raw_pressure()
        var1 = self.t_fine - 128000
        var2 = var1 * var1 * self.dig_P6
        var2 = var2 + ((var1 * self.dig_P5) << 17)
        var2 = var2 + (self.dig_P4 << 35)
        var1 = (((var1 * var1 * self.dig_P3) >> 8) + ((var1 * self.dig_P2) >> 12))
        var1 = (((1 << 47) + var1) * self.dig_P1) >> 33

        if var1 == 0:
            return 0
        
        p = 1048576 - raw
        p = (((p << 31) - var2) * 3125) // var1
        var1 = (self.dig_P9 * (p >> 13) * (p >> 13)) >> 25
        var2 = (self.dig_P8 * p) >> 19

        press = ((p + var1 + var2) >> 8) + (self.dig_P7 << 4)
        press = press // 256
        pressure = press / 100
        return pressure
    
    def altitude(self):
        p_0 = self.p0
        p = self.pressure()
        alt = 44307.69396 * (1 - ((p / p_0) ** 0.190284))
        return alt

    
#################################################################################################################
