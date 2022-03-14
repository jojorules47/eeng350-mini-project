// EENG350 Demo1 Functions

#include <Wire.h>

#define SLAVE_ADDRESS 0x04

int number;
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
    int a = 0;
    int address = Wire.read(); // address sent first
    
    distanceS = ""; // read in distance values until 'a'
    while(a != 'a') {
        a = Wire.read();
        if (a > 0) distanceS += (char)a;
      }
    
    angleS = ""; // read in angle values until 'b'
    while(a != 'b') {
      a = Wire.read();
      if (a > 0) angleS += (char)a;
    }
    
    distance = distanceS.toDouble();
    angle = angleS.toDouble();
    Serial.print("Distance: ");
    Serial.print(distance);
    Serial.print("   Angle: ");
    Serial.print(angle);
    Serial.println(' ');
  }
}


// callback for sending data
void sendData(){
  Wire.write((uint8_t *) &angle, 4);

}
