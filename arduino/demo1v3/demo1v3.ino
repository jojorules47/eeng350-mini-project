/**
 * Functions used in the feedback control of the robot. Include
 * these files for a "drop in" solution for robot controls.
 * 
 * TODO: Make solution work
 */

#include "controls.h"

// Define robot commands
#define DO_NOTHING 0
#define TURN 2
#define GO_FORWARD 1
#define FIND_TAPE 3

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
  double velocity = (currentPos - lastPos) / (double)SAMPLE_TIME * 1000.0;

  lastPos = currentPos;

  return velocity;
}

/**
 * Read robot angle
 */
double read_angle(){
  double radiansA = ((double)encA.read() * 2.0 * PI)/ 3200.0;
  double radiansB = -1.0*((double)encB.read() * 2.0 * PI)/ 3200.0;
  double phi = wheel_size*(radiansB - radiansA)/wheel_dist;
  return phi;
}

/**
 * Reset Encoders
 */
void reset_encoders(){
  encA.write(0);
  encB.write(0);
}

/**
 * PID Control loop. Adpated from example found here:
 * https://www.teachmemicro.com/arduino-pid-control-tutorial/
 * control_t struct tracks error terms
 */
double controller(double current, double target,
                  struct control_t &pid) {
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

/**
 * Controller with defined output and integral limits.
 */
double controller(double current, double target,double limit,
                  struct control_t &pid) {
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
double motor_speed(double velA, double velB, double distance, bool& done) {
  static double lastX = 0.0;
  static double lastY = 0.0;

  // Get forward velocity
  double rho_dot = wheel_size * (velA + velB) / 2.0;

  double target_vel = 0.5;
  double voltage = controller(rho_dot, target_vel, 3.0, forwardPID);

  double phi = read_angle();
  double nextX = lastX + SAMPLE_TIME/1000.0 * wheel_size* cos(phi)* (velA+velB)/2.0;
  double nextY = lastY + SAMPLE_TIME/1000.0 * wheel_size* sin(phi)* (velA+velB)/2.0;
  
  
  // Check if distance has been reached, and set flag
  if(nextX >= distance*0.96){
    voltage = 0.0;
    done = true;
    lastX = lastY = 0.0;
    nextX = nextY = 0.0;
    forwardPID.total_error = 0.0;
    Serial.print("Target acheived");
  }

  Serial.print("FORWARD: ");
  Serial.print(nextX);
  Serial.print("/");
  Serial.print(distance);
  Serial.print(" ");
  Serial.println(voltage);

  lastX = nextX;
  lastY = nextY;

  return voltage;
}

/**
 * Inner loop rotational velocity controller to control turning (phi_dot).
 */
double motor_direction(double velA, double velB, double turning, bool& done) {
  double current_angle = read_angle()/2;
  double target_phi = controller(current_angle, turning, 0.2, anglePID);

  double phi_dot = wheel_size*(velA - velB)/wheel_dist; // wheel_size, wheel_dist

  double voltage = controller(phi_dot, target_phi, 1.2, turningPID);

  Serial.print(" TURN: ");
  Serial.print(current_angle);
  Serial.print("/");
  Serial.print(turning);
  Serial.print(", ");
  Serial.println(voltage);

  // Check if target has been reached, and set flag
  if(turning != 0.0 && abs(current_angle - turning) <= 0.01){
    voltage = 0.0;
    done = true;
  }

  return voltage;
}

/**
 * Control motors to give robot specified forward speed and
 * rotational velocity using inner control loops. Place this function
 * in 'main' loop to control robot at a sampled time.
 */
bool motor_control(int &command, double target_distance, double target_angle) {
  static double lastX = 0.0;
  static double lastY = 0.0;
  static double phiOld = 0.0;
  static double motorA_pos = 0.0;
  static double motorB_pos = 0.0;

  bool done = false;
  
  double targetX = target_distance * cos(target_angle);
  double targetY = target_distance * sin(target_angle);

  double velA = -1.0 * read_motor(encA, motorA_pos);
  double velB = 1.0*read_motor(encB, motorB_pos);

  // Re-initialize when command is started
  if(first_run){
    velA = 0.0;
    velB = 0.0;
    first_run = false;
  }

  // Execute each state differently, using control to get motor voltages
  double forward_volts, turning_volts, fudge;
  switch(command){
    case DO_NOTHING:
      forward_volts = 0;
      turning_volts = 0;
      motorA_pos = motorB_pos = 0.0;
      done = true;
      break;

    case TURN:
      forward_volts = 0.0;
      turning_volts = motor_direction(velA, velB, target_angle, done);
      fudge = 0*0.059;
      break;

    case GO_FORWARD:
        forward_volts = motor_speed(velA, velB, target_distance, done);
        turning_volts = 0.0;
        fudge = 0*0.1;
      break;

    case FIND_TAPE:
      if(tape_found){
        command = DO_NOTHING;
        break;
      }
      forward_volts = 0.0;
      turning_volts = motor_direction(velA, velB, target_angle, done);
      break;
     
    default:
      Serial.println("ERROR");
  }

  // If a state finishes, and we're not doing nothing, tell Pi we've finished
  if(done && (command != DO_NOTHING)) {
    command = DO_NOTHING;
    ack = 1;
    Serial.println("bruh");
  }

  double voltsA = (forward_volts + turning_volts) / 2.0;
  double voltsB = (forward_volts - turning_volts) / 2.0;

  // Write voltages to motors
  analogWrite(speedA, volt_to_pwm(voltsA));
  digitalWrite(directionA, volt_to_dir(voltsA));

  analogWrite(speedB, volt_to_pwm(voltsB));
  digitalWrite(directionB, volt_to_dir(voltsB));

  return done;
}
