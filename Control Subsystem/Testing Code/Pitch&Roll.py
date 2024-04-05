import time
#import our sensor library
from PI4_sensor_library import QwiicKX13X

#Set up the accelerometer class and it will also set up the i2c bus 
Accelerometer = QwiicKX13X(1)

#Send values to configuration registers for the inital settings for measurement we want
Accelerometer.initialize(True, False, 2, False, False)
time.sleep(1)

#Print acceleration data readings 
while True:
    pitch, roll = Accelerometer.get_orientation()
    print(f'Pitch: {pitch} degrees, Roll: {roll} degrees')
    time.sleep(0.75)