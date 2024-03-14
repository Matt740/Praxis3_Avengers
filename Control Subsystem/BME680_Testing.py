import time
from PI4_sensor_library import BME680

Bme = BME680(1)

while True:
    if Bme.get_sensor_data():
        print('Pressure = ', Bme.data.pressure)
    else:
        print("bruh")