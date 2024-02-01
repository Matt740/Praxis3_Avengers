import machine
import utime

i2c = machine.I2C(0, scl=machine.Pin(17), sda=machine.Pin(16))

while True:
    devices = i2c.scan()
    if devices:
        for d in devices:
            print(hex(d))
    else:
        print("none")
    utime.sleep(0.5)