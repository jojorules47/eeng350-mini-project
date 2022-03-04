#include "DualMC33926MotorShield.h"
#include <Encoder.h>

Encoder encA(2, 5);  // Declare encoder object
Encoder encB(3, 6);  // Declare encoder object

const int SAMPLE_TIME = 5;
float voltage = 0;

int enablePin = 4; // enable pin, set LOW to turn off motor, set HIGH to turn on
int directionA = 7;  // direction motorA spins
int directionB = 8;  // direction motorB spins
int speedA = 9;      // speed of motorA, set by PWM
int speedB = 10;     // speed of motorB, set by PWM
int statusFlag = 12; // status flag, 0 if fault

int rotationA = LOW;      //initialize rotation direction of motorA, set LOW to spin opposite direction
int rotationB = LOW;      //initialize rotation direction of motorA, set LOW to spin opposite direction


void setup() {

  // pin setup
  pinMode(enablePin, OUTPUT);
  pinMode(directionA, OUTPUT);
  pinMode(directionB, OUTPUT);
  pinMode(speedA, OUTPUT);
  pinMode(speedB, OUTPUT);
  pinMode(statusFlag, INPUT);

  Serial.begin(115200);
  Serial.println("Voltage Speed Turning");

  digitalWrite(enablePin, HIGH);        //turn motor on
  digitalWrite(directionA, rotationA);  //set direction of motor
  digitalWrite(directionB, rotationB);  //set direction of motor
}

// Millis delay variables
int sleep = 1000;
int time_end = 3000;
unsigned long time_now = 0;

double r = 14.5/2; // Wheel radius
//long oldPosition = -999;  //initialize old count of encoder

void loop() {

  if(millis() >= sleep){
    voltage = 0.5;
  }

    int motor_speed = (int)(voltage / 5.0 * 255.0);

  analogWrite(speedA, motor_speed);              //set speed of motor
  analogWrite(speedB, motor_speed);              //set speed of motor

  analogWrite(speedA, motor_speed); // set speed of motor

  // Sample motor velocity, report over-run
  if (millis() >= time_now + SAMPLE_TIME) {
    time_now += SAMPLE_TIME;
    print_data();
    if(millis() > time_now+SAMPLE_TIME) Serial.println("Running Behind");
  }

  if(millis() >= time_end){
    analogWrite(speedA, 0);
    analogWrite(speedB, 0);
    while(1);
  }
}

void print_data(){
    static double motorA_pos = 0.0;
    static double motorB_pos = 0.0;
    double velA = -1.0*read_motor(encA, motorA_pos);
    double velB = read_motor(encB, motorB_pos);

    // Va = 2*voltage
    // Delta Va = 0*voltage
    Serial.print(voltage);
    Serial.print(", ");
    // rho_dot = r*(velA+velB)/2
    // phi_dot = r*(velA-velB)/2
    Serial.print(r*(velA+velB)/2.0);
    Serial.print(", ");
    Serial.println(r*(velA-velB)/2.0);
//   Serial.print(velA);
//   Serial.print(", ");
//   Serial.println(velB);

}

// double newRads = 0.00;     //initalize radians
// double lastRads = 0.00;
double read_motor(Encoder &enc, double &lastPos){
    long newPosition = enc.read();
    double currentPos = ((double)newPosition*2.0*PI)/3200.0;

    // Serial.print(voltage);
    // Serial.print(", ");
    // Serial.println((newRadsA-lastRadsA)/(SAMPLE_TIME)*1000.0);

    double velocity = (currentPos - lastPos)/(double)SAMPLE_TIME*1000.0;
//    Serial.print(currentPos);
//    Serial.print(", ");
//    Serial.print(lastPos);
//    Serial.print(", ");
//    Serial.print(velocity);
//    Serial.print(", ");
    lastPos = currentPos;

    return velocity;

}
