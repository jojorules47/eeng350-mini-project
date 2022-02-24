#include <Encoder.h>
#include "DualMC33926MotorShield.h"
#include <Wire.h>
#define SLAVE_ADDRESS 0x04
Encoder myEnc(2, 5);  // Declare encoder object

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

  pinMode(13, OUTPUT);
  Serial.begin(115200); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
 // Wire.onRequest(sendData);

  Serial.println("Ready!");

 // Serial.begin(9600);
  
}
int input = 0;
int angle = 2;


float newRads = 0.00;     //initalize radians
float oldRads = 0.00;
long oldPosition = -999;  //initialize old count of encoder
int clockWise = LOW; 
int counterClockWise = HIGH; //initialize rotation direction of motorA, set LOW to spin opposite direction


void loop() {
  digitalWrite(enablePin, LOW);        //turn motor on
//digitalWrite(directionA, leftRight);  //set direction of motor
  analogWrite(speedA, 50);   
  // **** Uncomment for serial monitor control ****
  if(Serial.available() > 0){   
    input = Serial.read();
  Serial.print(input);
  }
  
  //angles motor will spin to
  //change cases to 48, 49, 50, 51 for using serial monitor
  switch(input){
    case '0':
      newRads = 0.00;
      break;
    case '1':
      newRads = 1.57;
      break;
    case '2':
      newRads = 3.14;
      break;
    case '3':
      newRads = 4.71;
      break;
   // default:
     // newRads = 0.00;
      
  }
  if(newRads == oldRads){               //if desired angle equals current angle motor stays put
    digitalWrite(enablePin, LOW);
  }
  else if(oldRads < newRads){           // if desired angle is greater than current angle, rotate counter clockwise
    rotateCounterClockwise();
  }
  else if(oldRads > newRads){           // if desired angle is greater than current angle, rotate clockwise
    rotateClockwise();
  }
  
}

//rotates motor counter clockwise
void rotateCounterClockwise(){
  digitalWrite(directionA, counterClockWise);
  digitalWrite(enablePin, HIGH);
  while(oldRads <= newRads){
    long newPosition = myEnc.read(); 
    oldRads = (float)(2*newPosition*PI)/3200; //  read position of encoder
    if(newPosition != oldPosition){
      Serial.println(oldRads);
      oldPosition = newPosition;
    }
  }
  digitalWrite(enablePin, LOW);
  oldRads = newRads;
  //delay(500);
}

//rotates motor clockwise
void rotateClockwise(){
  digitalWrite(directionA, clockWise);
  digitalWrite(enablePin, HIGH);
  while(oldRads >= newRads){
    long newPosition = myEnc.read(); //  read position of encoder
    oldRads = (float)(2*newPosition*PI)/3200;  
    if(newPosition != oldPosition){
      Serial.println(oldRads);
      oldPosition = newPosition;
    }
  }
  digitalWrite(enablePin, LOW);
  oldRads = newRads;
 // delay(500);
}

void receiveData(int byteCount){

  Serial.print("received: ");
  while(Wire.available()) {

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
