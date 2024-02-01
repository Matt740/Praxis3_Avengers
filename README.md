# Praxis3_Avengers
The best of the best.

This document outlines the software files in our repository being used for our project

**I2C SENSORS***
When using any sensors with microcontrollers such as the Pico it is best to have one file which contains all the classes and code base for all the needed sensors.
In this project that file is sensors.py. if you are using a new sensor please add a class object for that sensor in the sensors.py file and follow along with how the other sensors class objects have been created. There are sensor object must have an __init__ function with the following parameters (self, bus=None, freq=None, sda=None, scl=None, network=None, addr=#Device I2C adress#). You can also include other parameters if you need them for that specific sensor. You'll notice that there are also two other classes at the top of sensor.py. These are for using i2C on the Pico and initializing I2C buses. Please use these in your definition of a new sensor's class object. You should be able to follow along with the BMP280 sensors init function for the I2C initialization. For any specific questions on I2C please reach out to Brody thanks. 

**Ensuring an I2C Device is Connected Properly***
If you are connecting a new sensor to a Pico and you want to make sure the sensor can communicate with the board please use Detect_i2c_Device.py. It scans your I2C SDA for attached devices and returns each device's address. This is always a good sanity check to do before running a main program. Note the scanner is currently set to use pins 16 and 17 so if you are using different I2C pins on the Pico you must change this in the code. In addition to telling you if the connection is made, the program returns the sensor address. This is useful if you do not know it as it will be needed to communicate with the sensor over I2C. 
