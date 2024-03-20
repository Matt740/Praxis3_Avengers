import time 
from PI4_sensor_library import L76B
from micropyGPS import MicropyGPS

Standby_Pin = 17
Force_Pin = 27

BAUDRATE = 9600
gps = L76B(Standby_Pin, Force_Pin)
gps.l76x_exit_backup_mode()
gps.l76x_send_command(gps.SET_NMEA_BAUDRATE_9600)
time.sleep(2)
gps.l76x_send_command(gps.SET_SYNC_PPS_NMEA_ON)
time.sleep(2)

#Not sure if I need this?????
#enable NMEA0183 sentence output
gps.l76x_send_command(gps.SET_NMEA_OUTPUT) 
time.sleep(2)

#set parser object
parser = MicropyGPS(location_formatting='dd')

while True:
    sentence = parser.update(chr(gps.uart_receive_byte()[0]))
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
