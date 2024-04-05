# Praxis3_Avengers
"Everybody wanted to know what we would do if it didn't fly, I guess we'll never know" **mic drop**

This document outlines the software files in our repository being used for our project. All of the code in this repository was used in the testing, prototyping, and final prototype of our Praxis 3 project. The repository is split into two main folders. These folders are for the two subsystems which require programming during our project. 

# Control Subsystem 
The control subsystem is in charge of controlling all sensors and external devices on the drone and must control flight and image detection processes. This is the most populated folder in our repository and it is split into two sub-folders. Those being **Testing Code** and **Device Libraries**.

**Testing Code** 
    The testing code folder contains all the programs that were written and used during testing of different compoenents and parts of our project. Each file is named approprialtely for what compoenent or functionlity they were testing. All of these files were uploadef at one point on the Raspberry Pi 4 and used for testing. 

**Device Libraries**
    This folder contains all the Pyhton classes we created for each external device used during the project and any other libraries that were needed for the project. The PI4_sensor_library contains classes for our accelrometer, barometer, GPS, motors, PiNoir Camera, and I2C. The microGPS file is used as a parser to convert NMEA sentences to readable longittude and lattitude. Finally the BME680_constant file contains all the ocnstant used in the barrometer class. All the classes defined in the files of this folder are called frrequentyl in testing files and  make testing code much easier to write and understand. 

# Detection Subsytem


