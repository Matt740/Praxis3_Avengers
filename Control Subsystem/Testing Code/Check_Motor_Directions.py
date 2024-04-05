import time
from PI4_sensor_library import Motor
import pigpio

PWM_PIN = 27

pi = pigpio.pi()
motor = Motor(pi, PWM_PIN)

motor.set_speed(0)
print('Speed set to zero')
time.sleep(4)

print('Starting Up')
motor.set_speed(5)
motor.set_speed(10)
motor.set_speed(15)

time.sleep(10)
motor.set_speed(0)
print('done')