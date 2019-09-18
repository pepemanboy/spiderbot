# Spiderbot Line Follower Robot
This repo hosts the code and electronics for my line following robot Spider Bot.
Spiderbot was developed in 2016, and competed in the international line following robots competition Robochallenge, hosted in Romania.
Our team was the 5th fastest team in the competition. (Click [here](https://www.youtube.com/watch?v=QOar2Q26z-w) to see video of Spiderbot in action).

Here is an image of the robot:

![Image of Spiderbot](https://github.com/pepemanboy/codigo-spiderbot/blob/master/pictures/spiderbot%20photo%20hd.png)

## Mechanical
Spiderbot was designed from scratch. Main characteristics are its 4 traction wheels with independent motors, and a ducted fan in the middle for improved traction.

## Electronics
Custom PCBs were developed for Spiderbot.
Schematics and PCB files are included in the electronics folder.

## Firmware
Simple firmware was developed for the robot, programmed in C and compiled with MPLABX for DSPIC30F5015.
Firmware implements simple PID controller, with adaptive gains that change based on a fuzzy logic algorithm.
IR sensors are used to estimate the position of the line, which is the input of the PID controller.
PID gains are changed by a fuzzy logic controller, which uses gyroscope data, which has information about the path that the robot is traversing (curve or straight line)

## Labview interface
I developed a labview interface to get/set PID parameters for the robot via bluetooth. 
It is located in the labview folder.
Labview 2013 was used.
