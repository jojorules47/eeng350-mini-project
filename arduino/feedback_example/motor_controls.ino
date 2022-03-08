#include "motor_controls.h"

Encoder encA(2, 5); // Declare encoder object
Encoder encB(3, 6); // Declare encoder object

int rotationA = LOW; // initialize rotation direction of motorA, set LOW to
                     // spin opposite direction
int rotationB = LOW; // initialize rotation direction of motorB, set LOW to
                     // spin opposite direction

struct control_t forwardPID = {1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0};
struct control_t turningPID = {1.0, 1.0, 1.0, 0.0, 0.0, 0.0, 0};

int volt_to_pwm(double volts) { return (int)(voltage / 5.0 * 255.0); }
bool volt_to_dir(double volts) { return (bool)(motor_voltage >= 0.0); }

double read_motor(Encoder &enc, double &lastPos) {
  long newPosition = enc.read();
  double currentPos = ((double)newPosition * 2.0 * PI) / 3200.0;

  double velocity = (currentPos - lastPos) / (double)SAMPLE_TIME * 1000.0;

  lastPos = currentPos;

  return velocity;
}

/*
 * PID Control loop. Adpated from example found here:
 * https://www.teachmemicro.com/arduino-pid-control-tutorial/
 */
// double controller(double target, double in) {
double controller(double current, double target_position,
                  struct control_t &pid) {
  // static unsigned long previousTime = 0;
  // static double total_error = 0, lastError = 0;

  unsigned long currentTime = millis(); // Get current time
  double elapsedTime =
      (double)(currentTime - pid.previousTime) / 1000.0; // Calculate interval

  pid.error = target - current;               // Determine current error
  pid.total_error += pid.error * elapsedTime; // Calculate integrated error
  double rate_error =
      (pid.error - pid.lastError) / elapsedTime; // Calculate derivative error

  double out = pid.p * error + pid.i * pid.total_error +
               pid.d * rate_error; // Total PID output

  pid.lastError = pid.error;
  pid.previousTime = currentTime;

  // Limit motor voltage, and prevent wind-up
  if (out > 5.0)
    out = 5.0;
  if (out < -5.0)
    out = -5.0;

  return out;
}

// TODO: Implement me
double motor_speed(double velA, double velB, double speed) {
  double rho_dot = wheel_size * (velA + velB) / 2.0;

  return controller(rho_dot, speed, forwardPID);
}
double motor_direction(double velA, double velB, double turning) {
  double phi_dot = wheel_size * (velA - velB) / wheel_dist;

  return controller(phi_dot, turning, turningPID);
}

void motor_control(double speed, double turning) {
  static double motorA_pos = 0.0;
  static double motorB_pos = 0.0;

  double velA = -1.0 * read_motor(encA, motorA_pos);
  double velB = read_motor(encB, motorB_pos);

  double forward_volts = motor_speed(velA, velB, speed);
  double turning_volts = motor_direction(velA, velB, turning);

  double voltsA = (forward_volts + turning_volts) / 2.0;
  double voltsB = (forward_volts - turning_volts) / 2.0;

  analogWrite(speedA, volt_to_pwm(voltsA));
  digitalWrite(directionA, volt_to_dir(voltA));

  analogWrite(speedB, volt_to_pwm(voltsB));
  digitalWrite(directionB, volt_to_dir(voltB));
}
