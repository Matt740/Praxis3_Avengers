import time
from PI4_sensor_library import Motor
import pigpio

PWM_PIN = 22

pi = pigpio.pi() 

motor = Motor(pi, PWM_PIN)
motor.set_speed(0)
print("Motor speed 0")
time.sleep(3)

for i in range(200):
    motor.set_speed(i)
    time.sleep(0.2)

feedback = None
while True:
    feedback = input("UP or Down or Exit? U/D/X: ")
    if feedback == "X":
        break
    else:
        speed = int(input("Wanted PWM signal change: "))
        if feedback == "U":
            motor.set_speed(motor.speed + speed)
        elif feedback == "D":
            motor.set_speed(motor.speed - speed)

for i in range(motor.speed, -1, -1):
    motor.set_speed(i)
    time.sleep(0.1)

    
    


