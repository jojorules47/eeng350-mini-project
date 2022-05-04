/**
 * Example showing feedback control of robot, to acheive
 * steady forward and rotational velocity using simple control
 * loops.
 */
 
#include "controls.h"
#define SLAVE_ADDRESS 0x04

unsigned long time_now = 0;


void setup() {

  // pin setup
  pinMode(enablePin, OUTPUT);
  pinMode(directionA, OUTPUT);
  pinMode(directionB, OUTPUT);
  pinMode(speedA, OUTPUT);
  pinMode(speedB, OUTPUT);
  pinMode(statusFlag, INPUT);

  digitalWrite(enablePin, HIGH);       // turn motor on
  digitalWrite(directionA, rotationA); // set direction of motor
  digitalWrite(directionB, rotationB); // set direction of motor
  
  Serial.begin(115200); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);
  Serial.println("Ready!");

}

//bool first = false;
// Millis delay variables
double target_vel = 0.0;
double target_turn = 0.0;
int sleep = 1000;

void loop() {
  // Step motor position to PI/2 after 1 second
  // Investigate removing this
//  if (millis() >= sleep && first == false) {
//    get_next_state();
//    first = true;
//  }




  // Read motor position, and determine motor voltage from PID controller
  // This is encapsulated in a function in `motorTest.ino`
  if (millis() >= time_now + SAMPLE_TIME) {
    time_now += SAMPLE_TIME;

    // Control motors to execute command camera_state
    bool done_yet = motor_control(camera_state, camera_distance, camera_angle);
    
    // Executes once robot has completed current command
    if(done_yet){
      get_next_state();
      reset_encoders();
    }
    
    if (millis() > time_now + SAMPLE_TIME)
      Serial.println("Took too long");
  }
}
