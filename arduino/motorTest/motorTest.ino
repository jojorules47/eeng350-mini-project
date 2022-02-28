#include "DualMC33926MotorShield.h"
#include <Encoder.h>
#include <Wire.h>
#define SLAVE_ADDRESS 0x04
Encoder myEnc(2, 5); // Declare encoder object

int enablePin = 4; // enable pin, set LOW to turn off motor, set HIGH to turn on
int directionA = 7;  // direction motorA spins
int directionB = 8;  // direction motorB spins
int speedA = 9;      // speed of motorA, set by PWM
int speedB = 10;     // speed of motorB, set by PWM
int statusFlag = 12; // status flag, 0 if fault

double controller(double, double);

void setup() {

  // pin setup
  pinMode(enablePin, OUTPUT);
  pinMode(directionA, OUTPUT);
  pinMode(directionB, OUTPUT);
  pinMode(speedA, OUTPUT);
  pinMode(speedB, OUTPUT);
  pinMode(statusFlag, INPUT);

  pinMode(13, OUTPUT);
  Serial.begin(115200); // start serial for output

  Serial.println("Position Voltage Error");
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  // Wire.onRequest(sendData);

  digitalWrite(enablePin, HIGH);
}
int input = 0;

double newRads = 0.00; // initalize radians
double oldRads = 0.00;
double motor_voltage = 0.0;

const int SAMPLE_TIME = 10;
unsigned long time_now = 0;

void loop() {
  // **** Uncomment for serial monitor control ****
  if (Serial.available() > 0) {
    // Subtract '0' for serial
    input = Serial.read() - '0';
    Serial.print(input);
  }
  // *** Case corresponds to position marker the camera reads ***
  switch (input) {
  case 0:
    newRads = 0.00;
    break;
  case 1:
    newRads = PI / 2;
    break;
  case 2:
    newRads = PI;
    break;
  case 3:
    newRads = 3 * PI / 2;
    break;
  }
  //** Sampling time **
  if (millis() >= time_now + SAMPLE_TIME) {
    time_now += SAMPLE_TIME;

    // Make motor go to newRads value
    drive_motor();

    if (millis() > time_now + SAMPLE_TIME)
      Serial.println("Took too long");
  }
}

void drive_motor() {
  long enc_counts = myEnc.read();
  oldRads = (double)(enc_counts * 2.0 * PI) / 3200;

  // Get PI controlled voltage
  motor_voltage = controller(newRads, oldRads);

  int motor_speed = (int)(abs(motor_voltage) / 5.0 * 255.0);

  double error = abs(newRads - oldRads);

  Serial.print(oldRads);
  Serial.print(" ");
  Serial.print(motor_voltage);
  Serial.print(" ");
  Serial.println(error);

  // Motor direction is counterclockwise (HIGH) if positive direction, clockwise
  // if negative
  digitalWrite(directionA, (motor_voltage >= 0.0)); // set direction of motor
  analogWrite(speedA, motor_speed);                 // set speed of motor
}

//** Communications between Pi and arduino **
void receiveData(int byteCount) {

  Serial.print("received: ");
  while (Wire.available()) {

    input = Wire.read();
    Serial.print(input);
    Serial.print(' ');

    // //block
    //    n = Wire.read();
    //    if (n != 0) {
    //      data[31-i] = n;
    //      Serial.println(data[31-i]);
    //      Serial.println(i);
    //      len++;
    //      i++;
    //    }
  }
  Serial.println(' ');
}

// ****** PI Controller config ****** //
// Less agressive
// const double kp = 5.0*0.24631;
// const double ki = 5.0*0.013906;

// Medium
const double kp = 5.0 * 0.36594;
const double ki = 5.0 * 0.029859;

// Aggresive
// const double kp = 5.0*0.52776;
// const double ki = 5.0*0.0594;

const double kd = 0.0;
/*
 * PID Control loop. Adpated from example found here:
 * https://www.teachmemicro.com/arduino-pid-control-tutorial/
 */
double controller(double target, double in) {
  static unsigned long previousTime = 0;
  static double total_error = 0, lastError = 0;

  unsigned long currentTime = millis(); // Get current time
  double elapsedTime =
      (double)(currentTime - previousTime) / 1000.0; // Calculate interval

  double error = target - in;         // Determine current error
  total_error += error * elapsedTime; // Calculate integrated error
  double rate_error =
      (error - lastError) / elapsedTime; // Calculate derivative error

  double out =
      kp * error + ki * total_error + kd * rate_error; // Total PID output

  lastError = error;
  previousTime = currentTime;

  // Limit motor voltage, and prevent wind-up
  if (out > 5.0)
    out = 5.0;
  if (out < -5.0)
    out = -5.0;

  return out;
}
