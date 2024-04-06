import time
from PI4_sensor_library import Motor, PiNoir, BME680, QwiicKX13X, L76B 
from micropyGPS import MicropyGPS
from ultralytics import YOLO
import pigpio


#Initialize Motors
pi = pigpio.pi() 
motor_FL = Motor(pi, 27)
motor_FR = Motor(pi, 22)
motor_RL = Motor(pi, 23)
motor_RR = Motor(pi, 24)
motors = [motor_FL, motor_FR, motor_RL, motor_RR]
print("Motors Initialized")

#Initialize Detection Subsytem 
camera = PiNoir("TrashPick.jpg")
model = YOLO("best.pt")
print("Detection System Initialized")

#Initialize Sensors
Accel = QwiicKX13X(1)
Accel.initialize(True, False, 2, False, False)

i2c_network = Accel.i2c

Barometer = BME680(1, network = i2c_network, sea_level_pressure=1017.9, sea_level_temperature=9.7)

parser = MicropyGPS(location_formatting='dd')
STANDBY = 26
FORCE = 19
BAUDRATE = 9600
gps = L76B(STANDBY, FORCE)
gps.l76x_exit_backup_mode()
time.sleep(2)
gps.l76x_send_command(gps.SET_NMEA_BAUDRATE_9600)
time.sleep(2)
print("Sensors Initialized")

motor_FL.set_speed(0)
motor_FR.set_speed(0)
motor_RL.set_speed(0)
motor_RR.set_speed(0)
print("Motor Speed Initialized to Zero")
time.sleep(3)

#Main Flight Loop Code
# Spin Up motors to speed that gets to nearly psoitve trust to weight
print("Starting Up Motors")
for i in range(20, 195):
    motor_FL.set_speed(i)
    motor_FR.set_speed(i*1.17)
    motor_RL.set_speed(i)
    motor_RR.set_speed(i)
    print("PWM signal at: ", i)
    time.sleep(0.25)
print("Initial Set Speed Reached")

# Run feedback loop which will take user input 
feedback = None
count = 0
while feedback != "X":
    feedback = input("Move Up or Down or Terminate? U/D/X: ")
    if feedback == 'X':
        break
    speed = int(input("Wanted Signal Change: "))
    if feedback == "U":
        new_speed = motor_FL.speed + speed 
        motor_FL.set_speed(new_speed)
        motor_FR.set_speed(new_speed*1.17)
        motor_RL.set_speed(new_speed)
        motor_RR.set_speed(new_speed)
    if feedback == "D":
        new_speed = motor_FL.speed - speed 
        motor_FL.set_speed(new_speed)
        motor_FR.set_speed(new_speed*1.17)
        motor_RL.set_speed(new_speed)
        motor_RR.set_speed(new_speed)

    # Capture Image and Run through model
    filename = 'Image{count}.jpg'
    if camera.capture_image(filename):
        print("Succesfully Took Image")
        results = model.predict(source=filename, save = True)
        count = count + 1

    #Collect sensor data
    x_a, y_a, z_a = Accel.get_accel_data()
    alt = Barometer.get_altitude()
    pressure = Barometer.data.pressure
    print("DATA READOUT\n ACCELEROMETER| X: ", x_a, "g, Y: ", y_a, "g, Z:", z_a, "g\n Barometer| Pressure: ", pressure, "hpa, Altitude: ", alt, "m")
    
    sentence = parser.update(chr(gps.uart_receive_byte()[0]))
    while not sentence:
        sentence = parser.update(chr(gps.uart_receive_byte()[0]))

    print("GPS|")
    print('WGS84 Coordinate:Latitude(%c),Longitude(%c) %.9f,%.9f'%(parser.latitude[1],parser.longitude[1],parser.latitude[0],parser.longitude[0]))
    print('UTC Timestamp:%d:%d:%d'%(parser.timestamp[0],parser.timestamp[1],parser.timestamp[2]))
    print('Horizontal Dilution of Precision:', parser.hdop)
    print('Satellites in Use by Receiver:',parser.satellites_in_use)

    print("-------------------------------------------------------------------------------------------------------------")

# Step down motors 
for i in range(motor_FL.speed - 1, -1, -1):
    motor_FL.set_speed(i)
    motor_FR.set_speed(i*1.17)
    motor_RL.set_speed(i)
    motor_RR.set_speed(i)
    time.sleep(0.1)












