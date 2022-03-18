/**
 * Functions used in the feedback control of the robot. Include
 * these files for a "drop in" solution for robot controls.
 * 
 * TODO: Make solution work
 */

#include "controls.h"

// Motor control utilities
int volt_to_pwm(double volts) { 
  double corr_volts = abs(volts);
  corr_volts = corr_volts >= 5.0 ? 5.0: corr_volts;
  return (int)(corr_volts / 5.0 * 255.0); }
bool volt_to_dir(double volts) { return (bool)(volts <= 0.0); }

/**
 * Read motor velocity from a given encoder,
 * using last recorded position to aggregate velocity.
 */
double read_motor(Encoder &enc, double &lastPos) {
  long newPosition = enc.read();
  double currentPos = ((double)newPosition * 2.0 * PI) / 3200.0;
  //double currentPos = ((double)newPosition * 360.0) / 3200.0;
  double velocity = (currentPos - lastPos) / (double)SAMPLE_TIME * 1000.0;

  lastPos = currentPos;

  return velocity;
}

double read_angle(){
  double radiansA = ((double)encA.read() * 2.0 * PI)/ 3200.0;
  double radiansB = -1.0*((double)encB.read() * 2.0 * PI)/ 3200.0;
  double phi = wheel_size*(radiansB - radiansA)/wheel_dist;
  return phi;
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
  double elapsedTime = (double)(currentTime - pid.previousTime) / 1000.0; // Calculate interval

  pid.error = target - current;               // Determine current error
  pid.total_error += pid.error * elapsedTime; // Calculate integrated error
  double rate_error = (pid.error - pid.last_error) / elapsedTime; // Calculate derivative error

  double out = pid.p * pid.error + pid.i * pid.total_error + pid.d * rate_error; // Total PID output

  pid.last_error = pid.error;
  pid.previousTime = currentTime;

  // Limit motor voltage, and prevent wind-up
  if (out > 5.0)
    out = 5.0;
  if (out < -5.0)
    out = -5.0;

  return out;
}

double controller(double current, double target,double limit,
                  struct control_t &pid) {
  // TODO: Investigate using constant sample time instead of current time for simplicity
  unsigned long currentTime = millis(); // Get current time
  double elapsedTime = (double)(currentTime - pid.previousTime) / 1000.0; // Calculate interval

  pid.error = target - current;               // Determine current error
  pid.total_error += pid.error * elapsedTime; // Calculate integrated error
  double rate_error =
      (pid.error - pid.last_error) / elapsedTime; // Calculate derivative error

  double out = pid.p * pid.error + pid.i * pid.total_error +
               pid.d * rate_error; // Total PID output

  pid.last_error = pid.error;
  pid.previousTime = currentTime;

  // Limit motor voltage, and prevent wind-up
  if (out > limit){
    out = limit;
    pid.total_error = limit;
  }
  if (out < -1.0*limit){
    out = -1.0*limit;
    pid.total_error = -1.0*limit;
  }

  return out;
}

/**
 * Inner loop velocity controller to control forward velocity (rho_dot).
 */
double motor_speed(double velA, double velB, double speed) {
//  double rho_dot = (velA + velB) / 2.0;
  double rho_dot = wheel_size * (velA + velB) / 2.0;

  

  return controller(rho_dot, speed, forwardPID);
}

/**
 * Inner loop rotational velocity controller to control turning (phi_dot).
 */
//double motor_direction(double velA, double velB, double turning) {
////  double phi_dot = (velA - velB);
//
//  double current_angle = read_angle();
//
//  double target_phi = controller(current_angle, turning, turningPID);
//  double my_phi_dot = wheel_size*(velA - velB)/wheel_dist; // wheel_size, wheel_dist
//
//  return controller(my_phi_dot, target_phi, turningPID);
//}

bool angle_done = false;
double motor_direction(double velA, double velB, double turning) {
//  double phi_dot = (velA - velB);
//double my_angle = PI/2;
  double current_angle = read_angle()/2;

  double target_phi = controller(current_angle, turning, 0.4, anglePID);
//  if(target_phi > 0.2) target_phi = 0.2;
  Serial.print(current_angle);
  Serial.print("/");
  Serial.print(turning);
  Serial.print(", ");
  Serial.println(target_phi);
  double phi_dot = wheel_size*(velA - velB)/wheel_dist; // wheel_size, wheel_dist

  double voltage = controller(phi_dot, target_phi, turningPID);
  if(turning != 0.0 && abs(current_angle) >= abs(turning)){
    voltage = 0.0;
    angle_done = true;
  }

    return voltage;
}
/**
 * Control motors to give robot specified forward speed and
 * rotational velocity using inner control loops. Place this function
 * in 'main' loop to control robot at a sampled time.
 */
 double targetPos = 3.0;
 double targetAngle = 0.0;
bool motor_control(double speed, double turning) {
  static double lastX = 0.0;
  static double lastY = 0.0;
  static double phiOld = 0.0;
  static double motorA_pos = 0.0;
  static double motorB_pos = 0.0;

  bool done = false;
  
  double targetX = targetPos * cos(targetAngle);
  double targetY = targetPos * sin(targetAngle);

  double velA = -1.0 * read_motor(encA, motorA_pos);
  double velB = 1.0*read_motor(encB, motorB_pos);

  double forward_volts, turning_volts, fudge;
  if(speed != 0.0){
    forward_volts = motor_speed(velA, velB, speed);
    turning_volts = 0.0;
    fudge = 0.08;
  } else{
    forward_volts = 0.0;
    turning_volts = motor_direction(velA, velB, turning);
    fudge = 0.0;
  }
    if(angle_done == true){
      done = true;
//      Serial.println("we done");
    }
  double voltsA = (forward_volts + turning_volts) / 2.0;
  double voltsB = (forward_volts - turning_volts) / 2.0;
  //double current_angle = read_angle();
  
  double nextX = lastX + SAMPLE_TIME/1000.0 * wheel_size* cos(phiOld)* (velA+velB)/2.0;
  double nextY = lastY + SAMPLE_TIME/1000.0 * wheel_size* sin(phiOld)* (velA+velB)/2.0;
  //double newPhi = phiOld + SAMPLE_TIME/1000.0 * (wheel_size/wheel_dist)*(velA-velB);
  //double newPhi = readAngle();
  if(speed != 0.0){
  if((nextX >= targetX*1.02)&&(nextY >= targetY*1.02)){
    voltsA = voltsB = 0.0;
    done = true;
  }
  Serial.print(nextX);
  Serial.print("/");
  Serial.print(targetX);
  Serial.print(" ");
  Serial.println(forwardPID.total_error);
  }
  lastX = nextX;
  lastY = nextY;
  
//  phiOld = phiNew;

//  Serial.print(voltsA);
//  Serial.print(", ");
//  Serial.println(voltsB);
//  Serial.print(lastX);
//  Serial.print(", ");
//  Serial.print(targetPos);
//  Serial.print(", ");
//   Serial.print(wheel_size*(velA+velB)/2.0,4);
//  Serial.print(", ");
//  Serial.print(wheel_size*(velA-velB)/wheel_dist,4);
//  Serial.print(" , ");
//  Serial.print(read_angle());
//  Serial.print(" , ");
//  Serial.println(targetAngle);

  analogWrite(speedA, volt_to_pwm(voltsA));
  digitalWrite(directionA, volt_to_dir(voltsA));

  analogWrite(speedB, volt_to_pwm(voltsB-fudge));
  digitalWrite(directionB, volt_to_dir(voltsB));

  return done;
}
