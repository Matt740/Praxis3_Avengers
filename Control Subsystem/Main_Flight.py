import time
from PI4_sensor_library import BME680, QwiicKX13X, L76B, Motor, PiNoir, set_motors
from micropyGPS import MicropyGPS
from ultralytics import YOLO
import pigpio


pi = pigpio.pi() 
motor_FL = Motor(pi, 27)
motor_FR = Motor(pi, 22)
motor_RL = Motor(pi, 23)
motor_RR = Motor(pi, 24)
motors = [motor_FL, motor_FR, motor_RL, motor_RR]

camera = PiNoir("TrashPick.jpg")
model = YOLO("YOLOv8n.pt")

motor_FL.set_speed(0)
motor_FR.set_speed(0)
motor_RL.set_speed(0)
motor_RR.set_speed(0)
print("Motor Speed Initialized to Zero")
time.sleep(3)

#Main Flight Loop Code
# Spin Up motors to speed that gets to nearly psoitve trust to weight
for i in range(210):
        if not set_motors(motors, i):
            break
        time.sleep(0.25)

# Run feedback loop which will take user input 
feedback = None 
while feedback != "end":
    feedback = input("UP or Down? U/D: ")
    speed = int(input("Wanted Signal Change: "))
    if feedback == "U":
        new_speed = motor_FL.speed + speed 
    if feedback == "D":
        new_speed = motor_FL.speed - speed 

# Step down motors 
for i in range(motor_FL.speed - 1, -1, -1):
    if not set_motors(motors, i):
        break
    time.sleep(0.1)












