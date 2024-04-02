import time
from PI4_sensor_library import BME680, QwiicKX13X, L76B, Motor, PiNoir
from micropyGPS import MicropyGPS
import pigpio

pi = pigpio.pi() 
motor_FL = Motor(pi, 27)
motor_FR = Motor(pi, 22)
motor_RL = Motor(pi, 23)
motor_RR = Motor(pi, 24)

camera = PiNoir("TrashPick.jpg")

motor_FL.set_speed(0)
motor_FR.set_speed(0)
motor_RL.set_speed(0)
motor_RR.set_speed(0)
time.sleep(3)

