/**
 * Functions used in the feedback control of the robot. Include
 * these files for a "drop in" solution for robot controls.
 * 
 * TODO: Make solution work
 */

#include "motor_controls.h"

// Motor control utilities
int volt_to_pwm(double volts) { 
  double corr_volts = abs(volts);
  corr_volts = corr_volts >= 5.0 ? 5.0: corr_volts;
  return (int)(corr_volts / 5.0 * 255.0); }
bool volt_to_dir(double volts) { return (bool)(volts <= 0.0); }
double sgn(double x) { return x >= 0.0 ? 1.0 : -1.0; }

/**
 * Read motor velocity from a given encoder,
 * using last recorded position to aggregate velocity.
 */
double read_motor(Encoder &enc, double &lastPos) {
  long newPosition = enc.read();
  double currentPos = ((double)newPosition * 2.0 * PI) / 3200.0;

  double velocity = (currentPos - lastPos) / (double)SAMPLE_TIME * 1000.0;

  lastPos = currentPos;

  return velocity;
}

/**
 * PID Control loop. Adpated from example found here:
 * https://www.teachmemicro.com/arduino-pid-control-tutorial/
 * control_t struct tracks error terms
 */
double controller(double current, double target,
                  struct control_t &pid) {
  // TODO: Investigate using constant sample time instead of current time for simplicity
  unsigned long currentTime = millis(); // Get current time
  double elapsedTime =
      (double)(currentTime - pid.previousTime) / 1000.0; // Calculate interval

  pid.error = target - current;               // Determine current error
  pid.total_error += pid.error * elapsedTime; // Calculate integrated error
  double rate_error =
      (pid.error - pid.last_error) / elapsedTime; // Calculate derivative error

  double out = pid.p * pid.error + pid.i * pid.total_error +
               pid.d * rate_error; // Total PID output

  pid.last_error = pid.error;
  pid.previousTime = currentTime;

  // Limit motor voltage, and prevent wind-up
  if (abs(out) > 10.0){
    out = sgn(out)*10.0;
    pid.error = sgn(pid.error)*min(10.0/pid.p, abs(pid.error));
    pid.total_error = (out-pid.p*pid.error)/pid.i;
  }

  return out;
}

double simple_controller(double current, double target,
                  struct control_t &pid) {
  pid.error = target - current;               // Determine current error
  double out = pid.p * pid.error;

  // Limit motor voltage, and prevent wind-up
  if (abs(out) > 10.0){
    out = sgn(out)*10.0;
  }

  return out;
}

/**
 * Inner loop velocity controller to control forward velocity (rho_dot).
 */
double motor_speed(double velA, double velB, double speed) {
//  double rho_dot = (velA + velB) / 2.0;
  double rho_dot = wheel_size * (velA + velB) / 2.0;

  return simple_controller(rho_dot, speed, forwardPID);
}

/**
 * Inner loop rotational velocity controller to control turning (phi_dot).
 */
double motor_direction(double velA, double velB, double turning) {
//  double phi_dot = (velA - velB);
  double phi_dot = wheel_size*(velA - velB)/wheel_dist; // wheel_size, wheel_dist

//  return controller(phi_dot, turning, turningPID);
    return simple_controller(phi_dot, turning, turningPID);
}

/**
 * Control motors to give robot specified forward speed and
 * rotational velocity using inner control loops. Place this function
 * in 'main' loop to control robot at a sampled time.
 */
 double targetPos = 5.0;
void motor_control(double speed, double turning) {
  static double lastX = 0.0;
  static double motorA_pos = 0.0;
  static double motorB_pos = 0.0;

  double velA = -1.0 * read_motor(encA, motorA_pos);
  double velB = read_motor(encB, motorB_pos);

  double forward_volts = motor_speed(velA, velB, speed);
  double turning_volts = motor_direction(velA, velB, turning);

  double voltsA = (forward_volts + turning_volts) / 2.0;
  double voltsB = (forward_volts - turning_volts) / 2.0;

  double nextX = lastX + SAMPLE_TIME/1000.0 * wheel_size*(velA+velB)/2.0;
  if(nextX >= targetPos){
    voltsA = voltsB = 0.0;
  }
  lastX = nextX;

//  Serial.print(voltsA);
//  Serial.print(", ");
//  Serial.print(voltsB);
  Serial.print(lastX);
  Serial.print(", ");
  Serial.print(targetPos);
  Serial.print(", ");
   Serial.print(wheel_size*(velA+velB)/2.0,4);
  Serial.print(", ");
  Serial.println(wheel_size*(velA-velB)/wheel_dist,4);

  analogWrite(speedA, volt_to_pwm(voltsA));
  digitalWrite(directionA, volt_to_dir(voltsA));

  analogWrite(speedB, volt_to_pwm(voltsB));
  digitalWrite(directionB, volt_to_dir(voltsB));
}
