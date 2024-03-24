import time
#import our sensor library
from PI4_sensor_library import QwiicKX13X

#Set up the accelerometer class and it will also set up the i2c bus 
Accelerometer = QwiicKX13X(1)

#Send values to configuration registers for the inital settings for measurement we want
Accelerometer.initialize(True, False, 2, False, False)
#Read value in g range cnfiguration register to see if it is the one we previously set
g_range = Accelerometer.get_g_range()
print(g_range)
if g_range == 2: print("Succesful I2C Read and Write")

# Prefrom hardware self check to test that accelerometer is giving accurate readings
if Accelerometer.run_command_test():
    print("Succeful Hardware Self Test")
else:
    print("Unceccesful Hardware Self Test")

time.sleep(4)

#Print acceleration data readings 
while True:
    x_accel, y_accel, z_accel = Accelerometer.get_accel_data()
    print(f'Acceleration X: {x_accel}, y: {y_accel}, z: {z_accel}')
    time.sleep(0.5)