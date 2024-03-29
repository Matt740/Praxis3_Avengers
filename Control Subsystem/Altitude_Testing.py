import time
from PI4_sensor_library import BME680
from BME680_constants import OS_1X

#Intitialize sensor and I2C class
# As well set sea level data which is obtained using local wheather station data
Bme = BME680(1, sea_level_pressure=10115, sea_level_temperature=4.9)

#Run loop that will get altitude by samppling pressure from BME6980 sensor and then use Barometric formula to return
#altitude in meters above sea level
while True:
    if Bme.get_altitude():
        print('Altitude =', Bme.data.altitude, 'm')
    else:
        print("failed to collect data")
    time.sleep(1)