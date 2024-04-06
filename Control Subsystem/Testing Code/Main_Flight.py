import time
from PI4_sensor_library import Motor, PiNoir, set_motors
from ultralytics import YOLO
import pigpio


pi = pigpio.pi() 
motor_FL = Motor(pi, 27)
motor_FR = Motor(pi, 22)
motor_RL = Motor(pi, 23)
motor_RR = Motor(pi, 24)
motors = [motor_FL, motor_FR, motor_RL, motor_RR]

camera = PiNoir("TrashPick.jpg")
model = YOLO("best.pt")

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

# Run feedback loop which will take user input 
feedback = None
count = 0
while feedback != "X":
    feedback = input("UP or Down? U/D/X: ")
    speed = int(input("Wanted Signal Change: "))
    if feedback == 'X':
        break
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
    filename = 'Image{count}.jpg'
    if camera.capture_image(filename):
        print("Succesfully Took Image")
        results = model.predict(source=filename, save = True)
        count = count + 1


# Step down motors 
for i in range(motor_FL.speed - 1, -1, -1):
    motor_FL.set_speed(i)
    motor_FR.set_speed(i*1.17)
    motor_RL.set_speed(i)
    motor_RR.set_speed(i)
    time.sleep(0.1)












