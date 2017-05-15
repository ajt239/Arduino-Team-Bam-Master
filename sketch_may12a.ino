#include <SoftwareSerial.h>

//define pins and create serial connection for bluetooth adapter 
//Pin 7 for TXD
//pin 8 for RXD 
SoftwareSerial bluetoothSerial(7,8);

void setup() {
  
  //start serial connection to bluetooth module 
  bluetoothSerial.begin(9600); 
  
  //start serial connection between arduino and computer 
  Serial.begin(9600); 
  Serial.println("Enter AT Commands for HC-08: ");
}

void loop() {
  //read input from HC-08 and print to serial monitor 
  if(bluetoothSerial.available()){
    Serial.write(bluetoothSerial.read()); 
  }

  //read input from Serial monitor and send it to the HC-08
  if(Serial.available()){
    bluetoothSerial.write(Serial.read()); 
  }
}
