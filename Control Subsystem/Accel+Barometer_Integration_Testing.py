import time
from PI4_sensor_library import BME680, QwiicKX13X

#Setup Accelerometer
Accelerometer = QwiicKX13X(1)
Accelerometer.initialize(True, False, 4, False, False)

# Get i2c_network from accelerometer to be used for other devices
i2c_network = Accelerometer.i2c

#Set up BME680
Barometer = BME680(1, i2c_network, 1015.1)

while True:
    x_a, y_a, z_a = Accelerometer.get_accel_data()
    alt = Barometer.get_altitude()
    pressure = Barometer.data.pressure()
    print("DATA READOUT\n ACCELEROMETER| X: ", x_a, "g, Y: ", y_a, "g, Z:", z_a, "g\n Barometer| Pressure: ", pressure, "hpa, Altitude: ", alt, "m \n -----------------------------------------------------------")