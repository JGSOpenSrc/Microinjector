# Microinjector
This repository contains the design files for a 3-D printable open hardware automated syringe pump, along with software to control the syringe pump over a serial port connection or via TTL logic. The original designs were published in a paper on the online journal PLOS ONE, and there are submodules of the relelvant repositories here.
# About
The design presented here is intended to allow for precise injection of liquid volumes from a high quality syringe with microliter precision (hence the name, microinjector). However the mechanical designs are easily adapted to syringes of other sizes (see the linear_actuator scripts or check out the submodules for more info).

The software in control runs on an Arduino in tandem with an Adafruit Motorshield (V2), using the motor shield library. Currently, calibration is performed by performing manual trials and tuning static constants in the arduino sketch to achieve desired levels of accuracy.

# Dependencies
- OpenSCAD is required to generate the models from the .scad files. You can download it from www.openscad.org.

- The Arduino IDE is required to compile and download the control script to the hardware (unless that is you have your own toolchain). Download it from www.arduino.c.

- The Adafruit Motor Shield e(VERSION 2!!) is needed to control the stepper motor used in the actuator. The best way to get it is grab it from github, to get it go to your arduino libraries folder and execute the command:

git clone https://github.com/adafruit/Adafruit_Motor_Shield_V2_Library Adafruit_Motorshield

# User's Guide #
The injector is controlled using the two limit switches, and either an open-drain (TTL) connection or a serial port connection (such as USB). The front limit switch is used to "prime" the injector, bringing it out of the reset state, and the rear limit switch is used to reset the position of the injector. The TTL pin and serial port are used to issue injection commands to the injector.

## Key Terms ##
- Zero-stroke: a configuration where the carriage is pressed up against the rear limit switch, at the motor end of the injector.
- Mid-stroke: a configuration where the carriage is in between the rear and front limit switch.
- Full-stroke: a configuration where the carriage is pressed up against the front limit switch, at the syringe end of the injector.
- Halt: a state where the injector is not primed and will ignore any injection commands. The halt state occurrs following reset, or once the injector reaches the zero-stroke or mid-stroke position
- Wait: a state where the injector is primed and will accept an injection command. The wait state occurrs when the front limit switch is pressed while in the halt state. Note, the injector must be in either the mid-stroke or zero-stroke position.

## Priming the Injector ##
After powering on, or following the assertion of reset, the injector will enter the halt state and will ignore any incoming injection request. To prime the injector, press the front limit switch once; this will put the injector in the wait state.

## Injecting via Serial Command ##
The serial port is configured run at 115200 baud, with 8 data bits, 1 stop bit, and null parity. To request an injection, send the string "inject" (without quotations) followed by a newline character.

## TTL Injection Trigger ##
The TTL pin interface is a two wire connection (signal and ground). An injection request is interpreted as a high-to-low transition on digital pin 5. The pin has an internal pull-up resistor and is 5V or 3V3 tolerant.

## Calibrating the IR Sensor ##
The injector uses an infrared emitter-detector pair to control the injection volume. Following an injection request, the motor will step forward until liquid is detected. While in the wait state, if the liquid remains for a certain amount of time then the injector will withdraw the liquid. This delay is set by a constant in the control.ino sketch.

The sensor consists of a photo-diode emitter and a photo-transistor detector. The detector is wired as an open-collector output; hence, when there is high IR absorption between the pair, the output of the sensor is a higher voltage, while when there is low absorption, the output of the sensor is a lower voltage.

For the sensor to work properly, two constants need to be calibrated in the control.ino sketch. 
- `const int high_to_low = ; // a low threshold, used to detect the presence of liquid`
- `const int low_to_high = ; // a high threshold, used to detect the absence of liqui`

Following reset, the injector will compute an average of ten IR readings, and determine the initial state of the liquid. If the average value is between the two thresholds, then the injector will not run (since it cannot determine the initial state). The average reading and the inferred initial state is printed to the serial port on every startup, and this output can be used to calibrate or troubleshoot the IR sensor.

## Wiring Configuration ##
For the injector to operate properly, there are three components that need to be connected to the arduino: the stepper motor, the limit switches, and the infrared liquid sensor.

### Motor Connection ###
The stepper motor uses a four wire bi-polar connection to the motor shield.

Motor Wire | Arduino Terminal
-----------|-----------------
Green      | M1
Black      | GND
Blue       | GND
Red        | M2

### Limit Switches ###
The limit switches connect to the motor shield by a split-end ribbon cable. The wires of the ribbon are numbered sequentially starting with the red wire being 1.

Ribbon Wire | Arduino pin
------------|------------
1           | GND
2           | Digital pin 10
3           | Digital pin 9

### IR Sensor ###
The infrared sensor is a 3 wire interface, consisting of 5V, GND, and an analog input.

Connector Position | Wire Color | Arduino Pin
-------------------|------------|------------
1                  | Black      | GND
2                  | White      | 5V
3                  | Grey       | Analog pin 0







