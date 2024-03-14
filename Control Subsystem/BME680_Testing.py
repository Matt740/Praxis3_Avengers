import time
from PI4_sensor_library import BME680

Bme = BME680(1, sea_level_pressure=1000)

while True:
    print(Bme.get_altitude())
    time.sleep(2)