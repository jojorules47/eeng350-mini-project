# Arduino Code and Examples

Required Libraries:

* `DualMC33926MotorShield.h` - Installed via Arduino IDE
* `Encoder.h` - Found on git [here](https://www.pjrc.com/teensy/td_libs_Encoder.html)

* `Wire.h` - Installed via Arduino IDE

<!-- ![](../img/motor_system.jpg) -->

## Final Demo Code

Code used for final demo, with commands, and combined controls to allow
for dynamic response. All contained within the `finaldemo/` Arduino project.

* `example.ino`: "Main" of program, constantly samples motor functions,
and sets up hardware.
* `comms.ino`: Contains ISR to handle I2C commands and update controls
targets.
* `controls.h`: Header defining globals, as well as PID controllers.
* `finaldemo.ino`: Contains the code that handles commands, and 
controls all motors.

## Examples

Contains all examples used to create the foundation of our final demo
code. This includes basic motor control, basic feedback controls, as well as basic communication examples.

## Old Demos

Older demo code is included for reference, and can be used as a
reference without needing to track git tag history.
