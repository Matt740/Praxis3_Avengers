# Praxis3_Avengers
"Everybody wanted to know what we would do if it didn't fly, I guess we'll never know" **mic drop**

This document outlines the software files in our repository being used for our project. All of the code in this repository was used in the testing, prototyping, and final prototype of our Praxis 3 project. The repository is split into two main folders. These folders are for the two subsystems which require programming during our project. 

# Control Subsystem 
The control subsystem is in charge of controlling all sensors and external devices on the drone and must control flight and image detection processes. This is the most populated folder in our repository and it is split into three sub-folders. Those being **Testing Code**, **Device Libraries** and **Old Code**.

**Testing Code** 
    The testing code folder contains all the programs that were written and used during testing of different compoenents and parts of our project. Each file is named approprialtely for what compoenent or functionlity they were testing. All of these files were uploadef at one point on the Raspberry Pi 4 and used for testing. 

**Device Libraries**
    This folder contains all the Pyhton classes we created for each external device used during the project and any other libraries that were needed for the project. The PI4_sensor_library contains classes for our accelrometer, barometer, GPS, motors, PiNoir Camera, and I2C. The microGPS file is used as a parser to convert NMEA sentences to readable longittude and lattitude. Finally the BME680_constant file contains all the ocnstant used in the barrometer class. All the classes defined in the files of this folder are called frrequentyl in testing files and  make testing code much easier to write and understand. 

**Old Code**
    This folder contains old code that was made for the control subsystem. This is remenat code from when we planned to use the Raspberry Pi Pico but as our computational demands changed we upgrade to the Raspeberry Pi 4 and thus ahd to chnage the code that was already written. 

# Detection Subsytem
The detection subsystem is in charge of retrieving images from a database, training the model based on the database, and testing the trained model with either some images found through the internet, or testing by connecting the model to the webcam, which allows real-time object detection.

In the detection subsystem folder, there are 5 code files, and two folders. The get_data file is used to download the dataset from the web to the local computer. The dataset is then used to train the model using the train_model file. After the model has been trained, the users can get the model's info (its class names) from the get_info file, and perform manual testing with either selected images or the webcam from the test files.

Some of the results of our tests are stored in runs/detect/Drone Testing. While the model is stored as best.pt in the filepath runs/detect/train16/weights.




