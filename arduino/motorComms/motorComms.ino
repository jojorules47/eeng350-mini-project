// EENG 350 Mini Project

#include <Wire.h>

#define SLAVE_ADDRESS 0x04


////E5
//int len = 0;
//byte data[32] = {0};
//int n=0, i=0;
int number;


void setup() {

  //E5
  pinMode(13, OUTPUT);
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

  Serial.print("received: ");
  while(Wire.available()) {

    number = Wire.read();
    Serial.print(number);
    Serial.print(' ');

// //block
//    n = Wire.read();
//    if (n != 0) {
//      data[31-i] = n;
//      Serial.println(data[31-i]);
//      Serial.println(i);
//      len++;
//      i++;
//    }
  }
  Serial.println(' ');
}

// callback for sending data
void sendData(){
  //E6
//  Wire.write(sensorValue);
  
//  //E5
//  for (int j=0; j<len; j++) {
//    data[len-1-j] = data[31-j];
//    Serial.println(data[j]);
//  }
//  Serial.println(data[31]);
//  Wire.write(data, len);
//  i = 0;
//  len = 0;
}
