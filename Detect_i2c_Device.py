
#Note obselete as doesn't workj for PI 4 please do not use

import machine
import utime

i2c = machine.I2C(0, scl=machine.Pin(17), sda=machine.Pin(16)) #Change these pins if you are using different I2C pins on your pico
                                                                # Ensure you have a scl and sda pin

while True:
    devices = i2c.scan()
    if devices:
        for d in devices:
            print(hex(d))
    else:
        print("none")
    utime.sleep(0.5)
