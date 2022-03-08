#ifndef __CONTROLS_H__
#define __CONTROLS_H__

#include "DualMC33926MotorShield.h"
#include <Encoder.h>

struct control_t {
  double p;
  double i;
  double d;
  double error;
  double total_error;
  double last_error;
  unsigned long previousTime;
};

const int SERIAL_RATE = 115200;
const int SAMPLE_TIME = 10;

const double wheel_size = 7.25;
const double wheel_dist = 10.0; // TODO: fixme

const int enablePin =
    4; // enable pin, set LOW to turn off motor, set HIGH to turn on
const int directionA = 7;  // direction motorA spins
const int directionB = 8;  // direction motorB spins
const int speedA = 9;      // speed of motorA, set by PWM
const int speedB = 10;     // speed of motorB, set by PWM
const int statusFlag = 12; // status flag, 0 if fault

// TODO: Need func defs?
int volt_to_pwm(double volts);
bool volt_to_dir(double volts);

double read_motor(Encoder &enc, double &lastPos);
double controller(double current, double target_position,
                  struct control_t &pid);

double motor_speed(double velA, double velB, double speed);
double motor_direction(double velA, double velB, double turning);
void motor_control(double speed, double turning);

#endif //__CONTROLS_H__