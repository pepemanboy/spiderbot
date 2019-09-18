# Spiderbot Line Follower Robot

This repo hosts the code and electronics for my line following robot Spider Bot.
Spiderbot was developed in 2016, and competed in the international line following robots competition Robochallenge, hosted in Romania.
Our team was the 5th fastest team in the competition.

## Electronics
Custom PCBs were developed for Spiderbot.
Schematics are included in the electronics folder.

## Firmware
Simple firmware was developed for the robot, programmed in C and compiled with MPLABX for DSPIC30F5015.
Firmware implements simple PID controller, with adaptive gains that change based on a fuzzy logic algorithm.
IR sensors are used to estimate the position of the line, which is the input of the PID controller.
PID gains are changed by a fuzzy logic controller, which uses gyroscope data, which has information about the path that the robot is traversing (curve or straight line)

## Labview interface
I developed a labview interface to get/set PID parameters for the robot via bluetooth. 
It is located in the labview folder.
Labview 2013 was used.