#include <Encoder.h>
#include "DualMC33926MotorShield.h"

Encoder myEnc(2, 5);  // Declare encoder object

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

  Serial.begin(115200);
  Serial.println("Encoder Test: ");

  digitalWrite(enablePin, HIGH);        //turn motor on
  digitalWrite(directionA, rotationA);  //set direction of motor
}

double target_position = 0.0;
int sleep = 1000;

void loop() {
  static double motor_voltage = 0.0;

  if(millis() >= sleep){
    target_position = PI/2;
  }

  if(millis() >= time_now+SAMPLE_TIME){
    time_now += SAMPLE_TIME;
    
    long enc_counts = myEnc.read();
    double pos_rads = (double)(enc_counts*2.0*PI)/3200;

    motor_voltage = controller(target_position, pos_rads);
    //motor_voltage += 0.1;
    int motor_speed = (int)(motor_voltage / 5.0 * 255.0);
    Serial.print(pos_rads);
    Serial.print(" ");
    double percent_error = abs(target_position-pos_rads)/target_position * 100.0;
    Serial.println(percent_error);
//    //analogWrite(speedA, (int)(2.5/5.0*255.0));
    analogWrite(speedA, motor_speed);
    digitalWrite(directionA, (motor_voltage >= 0.0));  //set direction of motor

    

    if(millis() > time_now+SAMPLE_TIME) Serial.println("Took too long");
  }
  
}

// Less agressive
//const double kp = 5.0*0.24631; //2.5*0.4522;
//const double ki = 5.0*0.013906; //2.5*0.1153;

// Medium
const double kp = 5.0*0.36594; //2.5*0.4522;
const double ki = 5.0*0.029859; //2.5*0.1153;

// Aggresive
//const double kp = 5.0*0.52776; //2.5*0.4522;
//const double ki = 5.0*0.0594; //2.5*0.1153;
const double kd = 0.0;


/*
 * PID Control loop. Adpated from example found here:
 * https://www.teachmemicro.com/arduino-pid-control-tutorial/
 */
double controller(double target, double in){
  static unsigned long previousTime = 0;
  static double total_error=0, lastError=0;
  
  unsigned long currentTime = millis(); // Get current time
  double elapsedTime = (double)(currentTime - previousTime)/1000.0; // Calculate interval

  double error = target - in; // Determine current error
  total_error += error * elapsedTime; // Calculate integrated error
  //double rate_error = (error - lastError)/elapsedTime; // Calculate derivative error

  double out = kp*error + ki*total_error; //+ kd*rate_error; // Total PID output

  lastError = error;
  previousTime = currentTime;

  //out += 0.0;
  if(out > 5.0) out = 5.0;

  return out;
}
