#include <Encoder.h>
#include "DualMC33926MotorShield.h"

Encoder encA(2, 5);  // Declare encoder object
Encoder encB(3, 6);  // Declare encoder object

const int SAMPLE_TIME = 10;
float voltage = 0;

int enablePin = 4;       // enable pin, set LOW to turn off motor, set HIGH to turn on
int directionA = 7;   //direction motorA spins
int directionB = 8;   // direction motorB spins
int speedA = 9;       //speed of motorA, set by PWM
int speedB = 10;      //speed of motorB, set by PWM
int statusFlag = 12;  //status flag, 0 if fault

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
  digitalWrite(directionA, 1^rotationA);  //set direction of motor
  digitalWrite(directionB, rotationB);  //set direction of motor
}

int sleep = 1000;
int time_end = 2000;
unsigned long time_now = 0;

double r = 0.2378; // Wheel radius, 7.25cm
double d = 0.7545; // Wheel dist, 23cm
//long oldPosition = -999;  //initialize old count of encoder

void loop() {

  if(millis() >= sleep){
    voltage = 1.0;
  }

  int motor_speed = (int)(voltage / 5.0 * 255.0);

  analogWrite(speedA, motor_speed);              //set speed of motor
  analogWrite(speedB, motor_speed);              //set speed of motor


  if(millis() >= time_now + SAMPLE_TIME){
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

// Nyquist: 60 us??

void print_data(){
    static double motorA_pos = 0.0;
    static double motorB_pos = 0.0;
    unsigned long timeA = 0;
    unsigned long timeB = 0;
    double velA = -1.0*read_motor(encA, motorA_pos, timeA);
    double velB = read_motor(encB, motorB_pos, timeB);

    // Va = 2*voltage
    // Delta Va = 0*voltage
    Serial.print(voltage);
    Serial.print(", ");
//    rho_dot = r*(velA+velB)/2
//     phi_dot = r*(velA-velB)/2
    Serial.print(r*(velA+velB)/2.0,4);
    Serial.print(", ");
    Serial.println(r*(velA-velB)/d,4);
//   Serial.print(velA,4);
//   Serial.print(", ");
//   Serial.println(velB,4);

}


double read_motor(Encoder &enc, double &lastPos, unsigned long &lastTime){
    long newPosition = enc.read();
    double currentPos = ((double)newPosition*2.0*PI)/3200.0;

    unsigned long currentTime = millis();
    double velocity = (currentPos - lastPos)*1000.0/ ((double)(currentTime - lastTime)); //(double)SAMPLE_TIME*1000.0;


    lastTime = currentTime;
    lastPos = currentPos;

    return velocity;

}
