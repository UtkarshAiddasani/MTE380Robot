# MTE380Robot

# Introduction
The objective of this project is to build and program a robot to handle a terrain with ramps, and magnetic walls to eventually reach a goal post. 

# Methodolody
As a team we decided to use magnets embedded into the wheels to drive up and down the magnetic wall. Then once over the wall we are required to analyze the area and locate a goal post. Thus 2 main sensors were selected: https://components101.com/sensors/mpu6050-module and https://www.digikey.ca/en/products/detail/parallax-inc/28995/3523692. The first is an IMU which could give information on orientation which is crucial to understand where the robot is and the second is an optical TOF sensor that can detect object 10-80 cm away which will allow for us to locate the goal post. The development board selected is an Arduino Uno.

# Algorithm
The algorithm is broken down into 2 main functions,getOverWall() for the getting over the wall movement and findPole for locating goal post. Other functions included are drive() which is used for driving straight, turnLeft and turnRight used for turning left and turning right and recieveGyro() to gather gyro readings as required.

# Results
Videos from the demo day can be seen here: https://drive.google.com/drive/folders/14_lJ4u-R2Wrcgvf9pqbziv9SB8DKR4qU?usp=drive_link


