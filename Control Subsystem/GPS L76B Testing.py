import time 
from PI4_sensor_library import L76B

Standby_Pin = 14
Force_Pin = 17

BAUDRATE = 9600
gps = L76B(Standby_Pin, Force_Pin)
gps.l76x_exit_backup_mode()
gps.l76x_send_command(gps.SET_SYNC_PPS_NMEA_ON)

while True:
    print(chr(gps.uart_receive_byte()[0]),end="")
    time.sleep(3)