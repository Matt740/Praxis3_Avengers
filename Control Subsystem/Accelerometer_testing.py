import time
from PI4_sensor_library import QwiicKX13X

Accelerometer = QwiicKX13X(1)
i2c_network = Accelerometer.i2c

Accelerometer.initialize(True, False, 8, False, False)
print(Accelerometer.get_g_range())

for i in range(10):
    x_accel, y_accel, z_accel = Accelerometer.get_accel_data()
    print(f'Acceleration X: {x_accel}, y: {y_accel}, z: {z_accel}')