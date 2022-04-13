#ifndef __CONTROLS_H__
#define __CONTROLS_H__
#include "DualMC33926MotorShield.h"
#include <Encoder.h>
#include <Wire.h>

/*** Type Definitions ***/
struct control_t {
  double p;
  double i;
  double d;
  double error;
  double total_error;
  double last_error;
  unsigned long previousTime;
};

/*** Define Constants ***/
const unsigned int SERIAL_RATE = 115200;
const int SAMPLE_TIME = 10;

const double wheel_size = 0.245; //7.25e-3;
const double wheel_dist = 0.443; //23.0e-3; // TODO: fixme

const int enablePin =
    4; // enable pin, set LOW to turn off motor, set HIGH to turn on
const int directionA = 7;  // direction motorA spins
const int directionB = 8;  // direction motorB spins
const int speedA = 9;      // speed of motorA, set by PWM
const int speedB = 10;     // speed of motorB, set by PWM
const int statusFlag = 12; // status flag, 0 if fault

/*** Define Controls Global Variables ***/
int rotationA = LOW; // initialize rotation direction of motorA, set HIGH to
                     // spin opposite direction
int rotationB = LOW; // initialize rotation direction of motorB, set HIGH to
                     // spin opposite direction
bool tape_found = false;
int ack = 0;
bool first_run = false;
Encoder encA(2, 5); // Declare encoder object
Encoder encB(3, 6); // Declare encoder object

// PID controllers
struct control_t forwardPID = {0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0};
struct control_t turningPID = {0.0,2.0, 0.0, 0.0, 0.0, 0.0, 0};
struct control_t anglePID = {0.0, 2.0, 0.0, 0.0, 0.0, 0.0, 0};


/*** Function Declarations ***/
int volt_to_pwm(double volts);
bool volt_to_dir(double volts);

double read_motor(Encoder &enc, double &lastPos);
double controller(double current, double target_position,
                  struct control_t &pid);

double motor_speed(double velA, double velB, double speed);
double motor_direction(double velA, double velB, double turning);
bool motor_control(int camera_state, float camera_distance, float camera_angle);
double read_angle();
void reset_encoders();

#endif //__CONTROLS_H__
