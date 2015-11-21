# Robotic arm software tools
This repository is dedicated for the project aimed to create a software tools necessary to control the robotic arm.

The main idea is to build and program the device that incorporates various sensors, like accelerometers and gyroscopes, which may allow the robotic arm to track its movement. 

In order to achieve the above mentioned goals the Freescale's FRDM-KL46Z board will be utilised.

# Team members:
- Dzierżewicz Tomasz,
- Kwiatosz Michał,
- Nawała Jakub.

# Functional assumptions

The main idea of the project is to integrate a robotic arm with the movement of our
hand. It is possible thanks to the motion controller with integrated gyroscope and
accelerometer.

## Operating diagram:
- booting the arm, passage to the standby mode after a short welcome gesture.
- robot is tracking position of the controller in the virtual 3D space and imitate its movement,
- arm is controlled by sensor with build-in accelerometer and gyroscope,
- if there is no motion robot performs a few of learned movements,
- at the end of tracking of our hand position robot is saying goodbye and goes to the starting position.

## Duties:
- handling the MPU-6050 sensor -> *Michał Kwiatosz*,
- creating the LabView-based communciation interface for the robotic arm -> *Tomasz Dzierżewicz*,
- creating the LabView-based control interface of the robotic arm -> *Jakub Nawała*.

## Base elements:
- Robotic Arm,
- FRDM KL46Z board,
- MPU-6050 module.

## Milestones
Date | Tomasz Dzierżewicz | Michał Kwiatosz | Jakub Nawała
-----|--------------------|-----------------|-------------
24 Nov 2015 | Application I/O | Library I/O and functionality | Application I/O
8 Dec 2015 | -- | -- | --
15 Dec 2015 | -- | -- | -- 
22 Dec 2015 | -- | -- | -- 
12 Jan 2016 | -- | -- | -- 
19 Jan 2016 | -- | -- | -- 
