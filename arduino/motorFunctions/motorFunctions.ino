#include <Encoder.h>
#include "DualMC33926MotorShield.h"
Encoder myEnc(2, 3);  // Declare encoder object

int enablePin = 4;       // enable pin, set LOW to turn off motor, set HIGH to turn on
int directionA = 7;   //direction motorA spins
int directionB = 8;   // direction motorB spins
int speedA = 9;       //speed of motorA, set by PWM
int speedB = 10;      //speed of motorB, set by PWM
int statusFlag = 12;  //status flag, 0 if fault
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
}

float newRads = 0.00;     //initalize radians
long oldPosition = -999;  //initialize old count of encoder
int rotationA = HIGH;      //initialize rotation direction of motorA, set LOW to spin opposite direction
void loop() {
  
  digitalWrite(enablePin, HIGH);        //turn motor on
  digitalWrite(directionA, rotationA);  //set direction of motor
  analogWrite(speedA, 35);              //set speed of motor

  //test I setup to turn wheel one full rotation one direction, then one full rotation the other direction
  while((newRads < 3.14) && (newRads > -3.14)){    
    long newPosition = myEnc.read();      //  read position of encoder
    if(newPosition != oldPosition){
      newRads = (float)(newPosition*PI)/3200;     // converts encoder position count to radians, 3200 counts per rotation
      Serial.println(newRads);
      oldPosition = newPosition;
    }
  }
  digitalWrite(enablePin, LOW);   //turn motor off
  delay(2000);
  myEnc.write(0);                 //reset encoder count
  newRads = 0;                    //reset radians count
  rotationA = !rotationA;           //toggle rotation direction
  
}
