#include <Encoder.h>
#include "DualMC33926MotorShield.h"

Encoder myEnc(2, 3);  // Declare encoder object

const int SAMPLE_TIME = 10;
float voltage = 0;

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

int sleep = 1000;
unsigned long time_now = 0;

float newRads = 0.00;     //initalize radians
long oldPosition = -999;  //initialize old count of encoder

void loop() {
  int motor_speed = (int)(voltage / 5.0 * 255.0);
  
//  digitalWrite(enablePin, HIGH);        //turn motor on
//  digitalWrite(directionA, rotationA);  //set direction of motor
  analogWrite(speedA, motor_speed);              //set speed of motor

  if(millis() >= sleep){
    voltage = 1.0;
  }

  if(millis() >= time_now + SAMPLE_TIME){
    time_now += SAMPLE_TIME;
    long newPosition = myEnc.read();
    newRads = (float)(newPosition*PI)/3200;
    Serial.println(newRads);
  }
  
  //test I setup to turn wheel one full rotation one direction, then one full rotation the other direction
//  while((newRads < 3.14) && (newRads > -3.14)){    
//    long newPosition = myEnc.read();      //  read position of encoder
//    if(newPosition != oldPosition){
//      newRads = (float)(newPosition*PI)/3200;     // converts encoder position count to radians, 3200 counts per rotation
//      Serial.println(newRads);
//      oldPosition = newPosition;
//    }
//  }
//  digitalWrite(enablePin, LOW);   //turn motor off
//  delay(2000);
//  myEnc.write(0);                 //reset encoder count
//  newRads = 0;                    //reset radians count
//  rotationA = !rotationA;           //toggle rotation direction
  
}
