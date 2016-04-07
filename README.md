# Microinjector
This repository contains the design files for a 3-D printable open hardware automated syringe pump, along with software to control the syringe pump over a serial port connection or via TTL logic. The original designs were published in a paper on the online journal PLOS ONE, and there are submodules of the relelvant repositories here.
# About
The design presented here intended to allow for precise injection of liquid volumes from a high quality syringe with microliter precision (hence the name, microinjector). However the mechanical designs are easily adapted to syringes of other sizes (see the linear_actuator scripts or check out the submodules for more info).

The software in control runs on an Arduino in tandem with an Adafruit Motorshield (V2), using the motor shield library. Currently, calibration is performed by performing manual trials and tuning static constants in the arduino sketch to achieve desired levels of accuracy.

# Dependencies
- OpenSCAD is required to generate the models from the .scad files. You can download it from www.openscad.org, or on linux grab it from apt-get as:
sudo apt-get install openscad
- The Arduino IDE is required to compile and download the control script to the hardware (unless that is you have your own toolchain). Download it from www.arduino.cc, or on linux grab it from apt-get as:
sudo apt-get install arduino
- The Adafruit Motor Shield e(VERSION 2!!) is needed to control the stepper motor used in the actuator. The best way to get it is grab it from github, to get it go to your arduino libraries folder and execute the command:
git clone https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library Adafruit_Motorshield

# How to use
A guide for using the microinjector is in the works.



