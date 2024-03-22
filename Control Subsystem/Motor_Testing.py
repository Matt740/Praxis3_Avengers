import time
from PI4_sensor_library import Motor
import pigpio

pi = pigpio.pi() 
motor1 = Motor(pi, 12)
motor1.set_speed(50)
time.sleep(3)
motor1.set_speed(0)