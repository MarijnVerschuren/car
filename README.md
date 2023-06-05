# CAR
This project builds on a 3D printed RC car project by making the ECU (electronic controll unit). (See the 3D RC car project for more information about the chassis and car componenets: https://cet3d.top/proto36-rc-car-posts/montaje-partes-3d-proto36/)
The ECU has a radio receiver which can receive data from a radio dongle that you can put into your laptop or PC. Under './controller' you can find the controller code written in python (in the current version you will need a PS3 controller to run this program).
The ECU also has a PID controler for traction controll to try and keep the car drivable at high speeds.

For a more technical overview of the system look at './receiver/README.md' here you will find images of the ECU, code snippets and peripheral usage tables.

## Sources / Sub-Projects
* https://cet3d.top/proto36-rc-car-posts/montaje-partes-3d-proto36/
* https://github.com/MarijnVerschuren/STM32F_CMSIS
* https://github.com/MarijnVerschuren/STM32F_CMSIS_RTOS
* https://github.com/MarijnVerschuren/Python_PS3
