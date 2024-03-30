import time 
from PI4_sensor_library import L76B
from micropyGPS import MicropyGPS


#Initialize parser object whihc will be used to Parse NMEA sentences from L76B
parser = MicropyGPS(location_formatting='dd')

# Set constant variables for pin numbers and Baudrate
STANDY_PIN = 17
FORCE_PIN = 27
BAUDRATE = 9600

#Initialize GPS class with force and stsandby pins
gps = L76B(STANDY_PIN, FORCE_PIN)

# Wake up GPS and then send it info over serial communication to configure it's on board baudrate
gps.l76x_exit_backup_mode()
gps.l76x_send_command(gps.SET_NMEA_BAUDRATE_9600)
time.sleep(2)

#Loop and keep recieving bytes over serial communication form the L76B and pur them through the parser to start forming lattitude and longittude
while True:
    sentence = parser.update(chr(gps.uart_receive_byte()[0]))
    #Once a full sentence has been parsed and is ready to be print this if statemnt will be entered and the info the parser has collected will be printed 
    if sentence:
            
        print('WGS84 Coordinate:Latitude(%c),Longitude(%c) %.9f,%.9f'%(parser.latitude[1],parser.longitude[1],parser.latitude[0],parser.longitude[0]))
        print('copy WGS84 coordinates and paste it on Google map web https://www.google.com/maps')
            
        print('UTC Timestamp:%d:%d:%d'%(parser.timestamp[0],parser.timestamp[1],parser.timestamp[2]))
            
        #print fix status
        '''
        1 : NO FIX
        2 : FIX 2D
        3 : FIX_3D
        '''
        print('Fix Status:', parser.fix_stat)
            
        print('Altitude:%d m'%(parser.altitude))
        print('Height Above Geoid:', parser.geoid_height)
        print('Horizontal Dilution of Precision:', parser.hdop)
        print('Satellites in Use by Receiver:', parser.satellites_in_use)
        print('')            
