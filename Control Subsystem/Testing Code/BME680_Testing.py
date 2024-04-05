import time
from PI4_sensor_library import BME680
from BME680_constants import OS_1X

#Intitialize sensor and I2C class
# As well set sea_level_pressure whihc is obtained using local wheather station data
Bme = BME680(1, sea_level_pressure=1014.34)


# Write to pressure oversmapling register
try:
    Bme.set_pressure_oversample(OS_1X)
except Exception:
    print("Failed to write to register")

# read from oversampling register
try: 
    print(Bme.get_pressure_oversample())
except Exception:
    print("Failed to read form register")

#Run loop that samples the pressure and prints out the value from the sensor
while True:
    if Bme.get_sensor_data():
        print('Pressure =', Bme.data.pressure, 'hpa')
    else:
        print("failed to collect pressure data")
    time.sleep(2)