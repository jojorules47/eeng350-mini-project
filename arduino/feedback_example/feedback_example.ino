#include <Encoder.h>
#include "DualMC33926MotorShield.h"

Encoder myEnc(2, 3);  // Declare encoder object

const int SAMPLE_TIME = 10;
unsigned long time_now = 0;

int enablePin = 4;       // enable pin, set LOW to turn off motor, set HIGH to turn on
int directionA = 7;   //direction motorA spins
int directionB = 8;   // direction motorB spins
int speedA = 9;       //speed of motorA, set by PWM
int speedB = 10;      //speed of motorB, set by PWM
int statusFlag = 12;  //status flag, 0 if fault

int rotationA = HIGH;      //initialize rotation direction of motorA, set LOW to spin opposite direction

void setup() {
  
 // pin setup
  pinMode(enablePin, OUTPUT);
  pinMode(directionA, OUTPUT);
  pinMode(directionB, OUTPUT);     
  pinMode(speedA, OUTPUT);
  pinMode(speedB, OUTPUT);
  pinMode(statusFlag, INPUT);

  Serial.begin(9600);
  Serial.println("Encoder Test: ");

  digitalWrite(enablePin, HIGH);        //turn motor on
  digitalWrite(directionA, rotationA);  //set direction of motor
}

double target_position = 1.0;

void loop() {
  static double motor_voltage = 0.0;

  if(millis() >= time_now+SAMPLE_TIME){
    time_now += SAMPLE_TIME;
    
    long enc_counts = myEnc.read();
    double pos_rads = (double)(enc_counts*PI)/3200;

    motor_voltage = controller(target_position, pos_rads);
    int motor_speed = (int)(motor_voltage / 5.0 * 255.0);
    Serial.print(pos_rads);
    Serial.print(" ");
    Serial.println(motor_voltage);
//    analogWrite(speedA, motor_speed);

    if(millis() > time_now+SAMPLE_TIME) Serial.println("Took too long");
  }
  
}

const double kp = 1.0;
const double ki = 0.0;
const double kd = 0.0;
/*
 * PID Control loop. Adpated from example found here:
 * https://www.teachmemicro.com/arduino-pid-control-tutorial/
 */
double controller(double target, double in){
  static unsigned long previousTime = 0;
  static double total_error=0, lastError=0;
  
  unsigned long currentTime = millis(); // Get current time
  double elapsedTime = (double)(currentTime - previousTime); // Calculate interval

  double error = target - in; // Determine current error
  total_error += error * elapsedTime; // Calculate integrated error
  double rate_error = (error - lastError)/elapsedTime; // Calculate derivative error

  double out = kp*error + ki*total_error + kd*rate_error; // Total PID output

  lastError = error;
  previousTime = currentTime;

  return out;
}
