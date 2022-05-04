
// EENG350 Demo1 Functions

#define SLAVE_ADDRESS 0x04

int pi_state = DO_NOTHING;

double pi_angle, pi_distance = 0.0;
int camera_state = DO_NOTHING;
double camera_angle = 0.0;
double camera_distance = 0.0;

// callback for received data
bool dataReady = false;
void receiveData(int byteCount) {
  String angleS, distanceS;
  while (Wire.available()) {
    int a = 0, s, t;
    int address = Wire.read(); // address sent first

    s = Wire.read() - '0'; // read in state
    while (a != 'a')
      a = Wire.read();

    distanceS = ""; // read in distance values until 'a'
    while (a != 'b') {
      a = Wire.read();
      if (a > 0)
        distanceS += (char)a;
    }

    angleS = ""; // read in angle values until 'b'
    while (a != 'c') {
      a = Wire.read();
      if (a > 0)
        angleS += (char)a;
    }
    t = Wire.read() - 48;
    pi_state = s;
    tape_found = t;

    switch (s) {
    case DO_NOTHING:
      if (camera_state == FIND_TAPE)
        tape_found = true;
      break;
    case GO_FORWARD:
      pi_distance = distanceS.toDouble();
      pi_angle = angleS.toDouble();
      break;
    case TURN:
      pi_angle = angleS.toDouble();
      break;
    case FIND_TAPE:
      pi_angle = angleS.toDouble();
      break;
    default:
      pi_state = 0;
    }
  }

  // Raise that new command is available
//  dataReady = true;
  camera_state = pi_state;
  camera_distance = pi_distance;

  // TODO: investigate cumulative angle
  camera_angle = camera_angle+pi_angle;
}

// callback for sending data
void sendData() {
  Serial.print("Sending: ");
  Serial.println(ack);
  Wire.write(ack);
}

/**
 * Determine next command for robot to execute, if a command is available
 * from the Pi
 */
 bool first_state = false;
void get_next_state() {
  // TODO: Remove debug statements
//  if(first_state == false){
//  dataReady = true;
//  pi_state = 1;
//  pi_distance = 10.0;
//  pi_angle = PI/2;
//  first_state = true;
//  }
  
  if (dataReady) {
    // Update command and targets
    camera_state = pi_state;
    camera_distance = pi_distance;
    camera_angle = pi_angle;

    // Reset robot values for next command
    ack = 0;
    first_run = true;
    dataReady = false;

    Serial.print("New State: ");
    Serial.print(pi_state);
    Serial.print(" Distance: ");
    Serial.print(camera_distance);
    Serial.print(" Angle: ");
    Serial.println(camera_angle);
  }
}
