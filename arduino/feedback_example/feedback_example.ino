#include "DualMC33926MotorShield.h"
#include <Encoder.h>

#include "motor_controls.h"

// Encoder encA(2, 5); // Declare encoder object

// const int SAMPLE_TIME = 10;
unsigned long time_now = 0;

// int enablePin = 4; // enable pin, set LOW to turn off motor, set HIGH to turn
// on int directionA = 7;  // direction motorA spins int directionB = 8;  //
// direction motorB spins int speedA = 9;      // speed of motorA, set by PWM
// int speedB = 10;     // speed of motorB, set by PWM
// int statusFlag = 12; // status flag, 0 if fault

// int rotationA = HIGH; // initialize rotation direction of motorA, set LOW to
//                       // spin opposite direction

void setup() {

  // pin setup
  pinMode(enablePin, OUTPUT);
  pinMode(directionA, OUTPUT);
  pinMode(directionB, OUTPUT);
  pinMode(speedA, OUTPUT);
  pinMode(speedB, OUTPUT);
  pinMode(statusFlag, INPUT);

  Serial.begin(SERIAL_RATE);
  // Serial.println("Encoder Test: ");

  digitalWrite(enablePin, HIGH);       // turn motor on
  digitalWrite(directionA, rotationA); // set direction of motor
  digitalWrite(directionB, rotationB); // set direction of motor
}

// Millis delay variables
double target_vel = 0.0;
double target_turn = 0.0;
int sleep = 1000;

void loop() {
  // static double motor_voltage = 0.0;

  // Step motor position to PI/2 after 1 second
  if (millis() >= sleep) {
    target_vel = 3.0;
  }

  // Read motor position, and determine motor voltage from PID controller
  // This is encapsulated in a function in `motorTest.ino`
  if (millis() >= time_now + SAMPLE_TIME) {
    time_now += SAMPLE_TIME;
    /*
        long enc_counts = myEnc.read();
        double pos_rads = (double)(enc_counts * 2.0 * PI) / 3200;

        // Determine motor voltage from controller
        motor_voltage = controller(target_position, pos_rads);
        int motor_speed = (int)(motor_voltage / 5.0 * 255.0);

        // Report some metrics
        Serial.print(pos_rads);
        Serial.print(" ");
        double percent_error =
            abs(target_position - pos_rads) / target_position * 100.0;
        Serial.println(percent_error);

        // Update motor voltage and direction
        analogWrite(speedA, motor_speed);
        digitalWrite(directionA, (motor_voltage >=
                                  0.0)); // set direction of motor from voltage
       sign
    */
    motor_control(target_vel, target_turn);
    if (millis() > time_now + SAMPLE_TIME)
      Serial.println("Took too long");
  }
}

// double read_motor(Encoder &enc, double &lastPos) {
//   long newPosition = enc.read();
//   double currentPos = ((double)newPosition * 2.0 * PI) / 3200.0;

//   double velocity = (currentPos - lastPos) / (double)SAMPLE_TIME * 1000.0;

//   lastPos = currentPos;

//   return velocity;
// }

// struct control_t forwardPID = {1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0};
// void drive_motor() {
//   long enc_counts = encA.read();
//   double current_pos = (double)(enc_counts * 2.0 * PI) / 3200;

//   // Get PI controlled voltage
//   double forward_voltage = controller(target_vel, current_pos, forwardPID);
//   // double voltageA = controller(newRads, oldRads);

//   int motor_speed = (int)(abs(motor_voltage) / 5.0 *
//                           255.0); // Motor voltage to PWM percentage

//   double error = abs(newRads - oldRads);

//   Serial.print(oldRads);
//   Serial.print(" ");
//   Serial.print(voltageA);
//   Serial.print(" ");
//   Serial.println(error);

//   // Motor direction is counterclockwise (HIGH) if positive direction,
//   clockwise
//   // if negative
//   digitalWrite(directionA, (motor_voltage >= 0.0)); // set direction of motor
//   analogWrite(speedA, motor_speed);                 // set speed of motor
// }

// // Less agressive
// // const double kp = 5.0*0.24631; //2.5*0.4522;
// // const double ki = 5.0*0.013906; //2.5*0.1153;

// // Medium
// const double kp = 5.0 * 0.36594;  // 2.5*0.4522;
// const double ki = 5.0 * 0.029859; // 2.5*0.1153;

// // Aggresive
// // const double kp = 5.0*0.52776; //2.5*0.4522;
// // const double ki = 5.0*0.0594; //2.5*0.1153;
// const double kd = 0.0;

// struct control_t {
//   double p;
//   double i;
//   double d;
//   double error;
//   double total_error;
//   double last_error;
//   unsigned long previousTime;
// };

// /*
//  * PID Control loop. Adpated from example found here:
//  * https://www.teachmemicro.com/arduino-pid-control-tutorial/
//  */
// // double controller(double target, double in) {
// double controller(double current, double target_position,
//                   struct control_t &pid) {
//   // static unsigned long previousTime = 0;
//   // static double total_error = 0, lastError = 0;

//   unsigned long currentTime = millis(); // Get current time
//   double elapsedTime =
//       (double)(currentTime - pid.previousTime) / 1000.0; // Calculate
//       interval

//   pid.error = target - current;               // Determine current error
//   pid.total_error += pid.error * elapsedTime; // Calculate integrated error
//   double rate_error =
//       (pid.error - pid.lastError) / elapsedTime; // Calculate derivative
//       error

//   double out = pid.p * error + pid.i * pid.total_error +
//                pid.d * rate_error; // Total PID output

//   pid.lastError = pid.error;
//   pid.previousTime = currentTime;

//   // Limit motor voltage, and prevent wind-up
//   if (out > 5.0)
//     out = 5.0;
//   if (out < -5.0)
//     out = -5.0;

//   return out;
// }
