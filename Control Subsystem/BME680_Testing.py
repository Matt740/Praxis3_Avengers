import time
from PI4_sensor_library import BME680

Bme = BME680(1, sea_level_pressure=1004.85)

while True:
    print('Altitude =', Bme.get_altitude(), 'm')
    print('Pressure =', Bme.data.pressure, 'hpa')
    time.sleep(2)