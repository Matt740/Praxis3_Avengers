import time
import random
from PI4_sensor_library import BME680, QwiicKX13X, L76B
from micropyGPS import MicropyGPS

#Setup Accelerometer
Accelerometer = QwiicKX13X(1)
Accelerometer.initialize(True, False, 4, False, False)

# Get i2c_network from accelerometer to be used for other devices
i2c_network = Accelerometer.i2c

#Set up BME680
Barometer = BME680(1, i2c_network, 1015.1)

#set up parser
GPS_Parser = MicropyGPS(location_formatting='dd')
if not GPS_Parser.start_logging("loggedGPS.txt"):
    print("Error opening File")

#Inialize GPS Varibales
STANDBY = 17
FORCE = 27
BAUDRATE = 9600
#Initialize GPS Module
gps = L76B(STANDBY, FORCE)
gps.l76x_exit_backup_mode()
time.sleep(2)
gps.l76x_send_command(gps.SET_NMEA_BAUDRATE_9600)
time.sleep(2)


while True:
    #Print readout form i2c sensors
    x_a, y_a, z_a = Accelerometer.get_accel_data()
    alt = Barometer.get_altitude()
    pressure = Barometer.data.pressure()
    print("DATA READOUT\n ACCELEROMETER| X: ", x_a, "g, Y: ", y_a, "g, Z:", z_a, "g\n Barometer| Pressure: ", pressure, "hpa, Altitude: ", alt, "m")

    # Update gps parser
    sentence = GPS_Parser.update(chr(gps.uart_receive_byte()[0]))
    if sentence:
        print("GPS|")
        print('WGS84 Coordinate:Latitude(%c),Longitude(%c) %.9f,%.9f'%(GPS_Parser.latitude[1],GPS_Parser.longitude[1],GPS_Parser.latitude[0],GPS_Parser.longitude[0]))
        print('UTC Timestamp:%d:%d:%d'%(GPS_Parser.timestamp[0],GPS_Parser.timestamp[1],GPS_Parser.timestamp[2]))
        print('Horizontal Dilution of Precision:', GPS_Parser.hdop)
        print('Satellites in Use by Receiver:',GPS_Parser.satellites_in_use)


    interrupt = random.randint(5,9)
    if interrupt == 7:
        GPS_Parser.write_log(GPS_Parser.latitude_string()+GPS_Parser.longitude_string())
        print("INTERUPTED - GPS Saved in File")

    print("------------------------------------------------------------------------------------")
    time.sleep(1)





