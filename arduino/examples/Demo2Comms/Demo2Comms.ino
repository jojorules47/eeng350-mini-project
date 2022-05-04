// EENG350 Demo1 Functions

#include <Wire.h>

#define SLAVE_ADDRESS 0x04

int number, state;
String angleS, distanceS;
double angle, distance;


void setup() {
  Serial.begin(115200); // start serial for output
  // initialize i2c as slave
  Wire.begin(SLAVE_ADDRESS);

  // define callbacks for i2c communication
  Wire.onReceive(receiveData);
  Wire.onRequest(sendData);

  Serial.println("Ready!");
}

void loop() {
  delay(100);
}


// callback for received data
void receiveData(int byteCount){

  while(Wire.available()) {
    int a = 0, s;
    int address = Wire.read(); // address sent first

    s = Wire.read() - 48; // read in state
    while(a != 'a') a = Wire.read();
    
    distanceS = ""; // read in distance values until 'a'
    while(a != 'b') {
        a = Wire.read();
        if (a > 0) distanceS += (char)a;
      }
    
    angleS = ""; // read in angle values until 'b'
    while(a != 'c') {
      a = Wire.read();
      if (a > 0) angleS += (char)a;
    }
    Serial.println(s);
    switch (s) {
      case 0:
        Serial.println("none");
        break;
      case 1:
        Serial.println("straight");
        distance = distanceS.toDouble();
        Serial.print(distance);
        break;
      case 2:
        Serial.println("rotate");
        angle = angleS.toDouble();
        Serial.print(angle);
        break;
      case 3:
        Serial.println("find");
        break;
    }
  }
}


// callback for sending data
void sendData(){
  Wire.write((uint8_t *) &angle, 4);

}
