import time
from PI4_sensor_library import Motor
import pigpio

#Initialize Pi4 software PWM class
pi = pigpio.pi()

#Initialize motor class using software PWM class
motor_FL = Motor(pi, 27)
motor_FR = Motor(pi, 22)
motor_RL = Motor(pi, 23)
motor_RR = Motor(pi, 24)

#Initilaly set PWM to 0
motor_FL.set_speed(0)
motor_FR.set_speed(0)
motor_RL.set_speed(0)
motor_RR.set_speed(0)
time.sleep(2)

#Set all PWM pins to 100 
motor_FL.set_speed(100)
motor_FR.set_speed(100)
motor_RL.set_speed(100)
motor_RR.set_speed(100)
# Give enough time ti check voltages accross eahc PWM
time.sleep(90)

# Set all PWM pins back to zero
motor_FL.set_speed(0)
motor_FR.set_speed(0)
motor_RL.set_speed(0)
motor_RR.set_speed(0)