#include <Encoder.h>
#include "DualMC33926MotorShield.h"

Encoder myEnc(2, 5);  // Declare encoder object

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

  Serial.begin(115200);
  Serial.println("Encoder Test: ");

  digitalWrite(enablePin, HIGH);        //turn motor on
  digitalWrite(directionA, rotationA);  //set direction of motor
}

int sleep = 1000;
unsigned long time_now = 0;


//long oldPosition = -999;  //initialize old count of encoder

void loop() {

  if(millis() >= sleep){
    voltage = 5.0;
  }

    int motor_speed = (int)(voltage / 5.0 * 255.0);
  
  analogWrite(speedA, motor_speed);              //set speed of motor


  if(millis() >= time_now + SAMPLE_TIME){
    time_now += SAMPLE_TIME;
    read_motor();
    if(millis() > time_now+SAMPLE_TIME) Serial.println("Running Behind");
  }

  if(millis() >= 4000){
    analogWrite(speedA, 0);
    while(1);
  }
}

float newRads = 0.00;     //initalize radians
float lastRads = 0.00;
void read_motor(){
    long newPosition = myEnc.read();
    newRads = (float)(newPosition*2.0*PI)/3200;
//    if(newRads >= PI){
//      analogWrite(speedA, 0);
//      while(1);
//    }
//    Serial.println(newRads);
    Serial.print(voltage);
    Serial.print(", ");
    Serial.println((newRads-lastRads)/(SAMPLE_TIME)*1000.0);
    lastRads = newRads;
}
