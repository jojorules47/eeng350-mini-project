
// EENG350 Demo1 Functions

//#include <Wire.h>

#define SLAVE_ADDRESS 0x04

int pi_state = DO_NOTHING;

double pi_angle, pi_distance = 0.0;
int camera_state = DO_NOTHING;
double camera_angle = 0.0;
double camera_distance = 0.0;
bool ack = false;

// void setup() {
//   Serial.begin(115200); // start serial for output
//   // initialize i2c as slave
//   Wire.begin(SLAVE_ADDRESS);
//
//   // define callbacks for i2c communication
//   Wire.onReceive(receiveData);
//   Wire.onRequest(sendData);
//
//   Serial.println("Ready!");
// }

// void loop() {
//   delay(100);
// }

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
    // Serial.println(t);
    pi_state = s;
    tape_found = t;

    switch (s) {
    case DO_NOTHING:
      // Serial.println("none");
      if (camera_state == FIND_TAPE)
        tape_found = true;
      break;
    case GO_FORWARD:
      // Serial.println("straight");
      pi_distance = distanceS.toDouble();
      //  Serial.print(pi_distance);
      break;
    case TURN:
      // Serial.println("rotate");
      pi_angle = angleS.toDouble();
      // Serial.print(pi_angle);
      break;
    case FIND_TAPE:
      // Serial.println("find");
      pi_angle = angleS.toDouble();
      break;
    default:
      pi_state = 0;
      // Serial.print("Out of bounds");
    }
  }
  dataReady = true;
}

// callback for sending data
void sendData() {
  Serial.println("Sending ack...");
  Wire.write(1);
}

void get_next_state() {
  //  int next_state = 0;
  //  if(Serial.available() > 0){
  //    next_state = Serial.read() - '0';
  ////    Serial.print(input);
  //  } else{
  //    next_state = last_state;
  //  }
  //
  //  if(next_state < 0 || next_state > 3){
  //    Serial.print("State not recognized: ");
  //    Serial.println(next_state);
  //
  //        next_state = 0;
  //  }

  // static int last_state = 0;
  //  if(camera_state != pi_state){
  //
  //  }
  // bool updated = (camera_distance != pi_distance) || (camera_angle !=
  // pi_angle);

  // pi_angle = 0.0;
  // pi_distance = 0.0;
  // pi_state = 2;
  // dataReady = true;

  if (dataReady) {
    camera_state = pi_state;
    camera_distance = pi_distance;
    camera_angle = pi_angle;
    sendData();
    dataReady = false;

    // Serial.print("Current State: ");
    // Serial.print(camera_state);
    Serial.print("New State: ");
    Serial.print(pi_state);
    Serial.print(" Distance: ");
    Serial.print(camera_distance);
    Serial.print(" Angle: ");
    Serial.println(camera_angle);
    // last_state = camera_state;
  }

  //  return next_state;
}

// void send_ack(){
//   Wire.beginTransmission()
//   Wire.write(true)
//   Wire.endTransmission()
// }

void update_target(int state) {
  switch (state) {
  case TURN:
    camera_angle = PI / 2;
    break;
  case GO_FORWARD:
    camera_distance = 3.0;
    break;
  }
}
