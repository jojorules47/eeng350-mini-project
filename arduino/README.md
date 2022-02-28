# Arduino Code and Examples

Required Libraries:

* `DualMC33926MotorShield.h` - Installed via Arduino IDE
* `Encoder.h` - Found on git [here](https://www.pjrc.com/teensy/td_libs_Encoder.html)

![](../img/motor_system.jpg)

## Integration Utilities

EENG350MP

## Feedback Control Utilities and Examples

`control.ino`: Utility used to find controls transfer function. Performs
a step response experiment, and reports velocity and voltage in a CSV
format. This script was used to derive the MATLAB simulation results
using a 5V step input.

`feedback_example.ino`: This file served as a sandbox for developing 
the PID feedback control loop used in our final `motorTest.ino`
project. Several controller types are provided in the comments, 
allowing for different motor responses. This script allows the user to
measure the controller error, which may be helpful in testing feedback
modifications.

## Localization Examples

The motorTest file uses two external libraries to drive the motor, `Encoder.h` and `DualMC33926MotorShield.h`.

The `Encoder.h` library is a high performance encoder reader that reads the current position of the motor. The object **Encoder myEnc(2,5)** uses pins 2 and 5 to keep track of position. Using **myEnc.read()** the current position of the motor is read and is converted to radians by multiplying by **(2PI/ 3200)**, 3200 being the number of encoder counts per rotation. Using I2C, the arduino recieves the desired position of the motor from the raspberry pi. Sending the current position and the target position through the **controller()** function, the motor rotates to the target position using the **drive_motor()** function, which is sampled every 10 microseconds. 

The `DualMC33926MotorShield.h` library uses pins 7 and 9 to control the speed and direction of the motor, respectively. The speed is controlled using **analogWrite(speedA, motor_speed);** which writes a PWM value to pin 7. The direction is controlled using **digitalWrite(directionA, (motor_voltage >= 0.0));**, which will rotate clockwise if **motor_voltage** is negative, and counter-clockwise if **motor_voltage** is positive. 
