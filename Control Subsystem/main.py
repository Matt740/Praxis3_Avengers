import time
from PI4_sensor_library import BME680, QwiicKX13X, L76B

Accelerometer = QwiicKX13X(1)
i2c_network = Accelerometer.i2c
Barometer = BME680(1, i2c_network)

Accelerometer.initialize(True, False, 8, False, False)
print(Accelerometer.get_accel_data())