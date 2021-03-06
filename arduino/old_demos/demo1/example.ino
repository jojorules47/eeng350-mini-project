/**
 * Example showing feedback control of robot, to acheive
 * steady forward and rotational velocity using simple control
 * loops.
 */
 
#include "controls.h"

unsigned long time_now = 0;

void setup() {

  // pin setup
  pinMode(enablePin, OUTPUT);
  pinMode(directionA, OUTPUT);
  pinMode(directionB, OUTPUT);
  pinMode(speedA, OUTPUT);
  pinMode(speedB, OUTPUT);
  pinMode(statusFlag, INPUT);

  Serial.begin(115200);

  digitalWrite(enablePin, HIGH);       // turn motor on
  digitalWrite(directionA, rotationA); // set direction of motor
  digitalWrite(directionB, rotationB); // set direction of motor
}

bool first = false;
// Millis delay variables
double target_vel = 0.0;
double target_turn = PI;
int sleep = 1000;

void loop() {
  // Step motor position to PI/2 after 1 second
  if (millis() >= sleep && first == false) {
    target_vel = 0.0;
    target_turn = PI;
    first = true;
  }

  // Read motor position, and determine motor voltage from PID controller
  // This is encapsulated in a function in `motorTest.ino`
  if (millis() >= time_now + SAMPLE_TIME) {
    time_now += SAMPLE_TIME;

    // Control motors to move in a straight line. See `motor_controls.ino`
    Serial.print(target_turn);
    Serial.print(", ");
    Serial.print(target_vel);
    Serial.print(" ");
    bool done_yet = motor_control(target_vel, target_turn);
          
    if(done_yet){
      target_turn = 0.0;
      target_vel = 0.1;
    }
    
    if (millis() > time_now + SAMPLE_TIME)
      Serial.println("Took too long");
  }
}
