#include <SoftwareSerial.h>

//define pins and create serial connection for bluetooth adapter 
//Pin 7 for TXD
//pin 8 for RXD 
SoftwareSerial bluetoothSerial(7,8); 

//setting Pin numberss for motor one.
//const int because it is not meant to change.  
const int motorOneDirectionPin = 8;
const int pwmMotorOnePin = 9; 

//setting pin numbers for motor 2 
//const int because it is not meant to change.
const int motorTwoDirectionPin = 10;
const int pwmMotorTwoPin = 11; 

//setting pin numbers for joy stick.
//const int because it is not meant to change. 
const int joystickForwardPin = 2; 
const int joystickBackwardPin = 3; 
const int joystickRightPin = 5; 
const int joystickLeftPin = 4; 

//set pin if atari joystick is used
//this pin is to take input from fire button
const int atari2600FireButtonPin = 6; 

//variables to help with controlling motor speed once bluetooth is integratsed 
int motorOneSpeed;
int motorTwoSpeed;

//Variable to count how many times the emergency brake button is pushed.
int emergencyBrakeCounterButton;
int emergencyBrakeCounterBluetooth;

void setup() {
  //begin Serial for troubleshooting until bluetooth is working 
  Serial.begin(9600); 
  bluetoothSerial.begin(9600); 

  //setting pins for motor 1 to output
  //this is done to sent signals to motor drivers
  pinMode(motorOneDirectionPin, OUTPUT); 
  pinMode(pwmMotorOnePin, OUTPUT); 
  Serial.println("MotorOne pins set to OUTPUT.");
  
  //setting pins for motor 1 to output
  //this is done to sent signals to motor drivers
  pinMode(motorTwoDirectionPin, OUTPUT); 
  pinMode(pwmMotorTwoPin, OUTPUT);
  Serial.println("MotorTwo pins set to OUTPUT."); 

  //Setting pins for joystick and buttons to input
  //this is for the arduino to read what the user is inputting so we can send the proper outputs. 
  pinMode(joystickForwardPin, INPUT_PULLUP);
  pinMode(joystickBackwardPin, INPUT_PULLUP); 
  pinMode(joystickRightPin, INPUT_PULLUP); 
  pinMode(joystickLeftPin, INPUT_PULLUP);
  Serial.println("Joystick pins set to INPUT_PULLUP.");

  pinMode(atari2600FireButtonPin, INPUT_PULLUP);
  Serial.println("Atari fire pin set to INPUT_PULLUP");

}

void loop() {
  
  //setup variables that will read the state of the input pins. 
  int forwardPinState = digitalRead(joystickForwardPin);
  int backwardPinState = digitalRead(joystickBackwardPin); 
  int rightPinState = digitalRead(joystickRightPin); 
  int leftPinState = digitalRead(joystickLeftPin); 
  int fireButtonPinState = digitalRead(atari2600FireButtonPin);

  /*
   * This block of code is just for troubleshooting if there are issues 
   * To take advantage of it hook the arduino up to the computer using a 
   * usb A male to B male cable.
   * Then open Arduino ide
   * Then click on "Tools" -> "Serial Monitor"
   * Then make sure that 9600 baud is selected in the bottom right. 
   */
  //Send value of the pins states for troubleshooting and testing. 
  //first print line to create space between setup line when loop first runs. 
  Serial.println(""); 
  Serial.println("Pin states:");
  Serial.print("Forward pin state: ");
  Serial.println(forwardPinState); 
  Serial.print("Backward pin state: ");
  Serial.println(backwardPinState); 
  Serial.print("Right pin state: ");
  Serial.println(rightPinState); 
  Serial.print("Left pin state: "); 
  Serial.println(leftPinState); 
  Serial.print("Fire button state: ");
  Serial.println(fireButtonPinState);
  delay(1);

  int bluetoothData;
  if (bluetoothSerial.available()) {
    bluetoothData = bluetoothSerial.read();  
    Serial.println("Input from Bluetooth: ");
    if(bluetoothData == 10){
      emergencyBrakeCounterBluetooth++;
    }else if(emergencyBrakeCounterBluetooth%2 == 1){
      emergencyBrakeBluetooth();
    }else if (bluetoothData == 0){
      // Non-zero input means "turn on LED".
      Serial.println("  on");
      brake();
    }else if (bluetoothData == 1){
      forward();
    }else if (bluetoothData == 2){
      backward();
    }else if(bluetoothData == 3){
      left();
    }else if(bluetoothData ==4){
      right();
    }else{
      // Input value zero means "turn off LED".
      Serial.println("  off");
      brake(); 
    }  
  }else{
    if(emergencyBrakeCounterBluetooth%2 == 1){
      emergencyBrakeBluetooth();
    }else if(fireButtonPinState == 0){
      emergencyBrakeCounterButton++;
    }else if(emergencyBrakeCounterButton%2 == 1){
      emergencyBrakeButton();
    }else if(forwardPinState == 0){
      if(leftPinState == 0){
        leftForward(); 
      }else if (rightPinState == 0){
        rightForward();
      }else{
      forward();
      }
    }else if (backwardPinState == 0){
      if(leftPinState == 0){
        leftBackward(); 
      }else if (rightPinState == 0){
        rightBackward();
      }else{
      backward();
      }
    }else if (rightPinState == 0){
      right();
    }else if(leftPinState == 0){
      left();
    }else{
      brake();
    }
  }    
}

//Function to gradually accelerate Motors
//Function to call to make car go forward
void forward(){
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, LOW);
  analogWrite(pwmMotorOnePin,255); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin,LOW); 
  analogWrite(pwmMotorTwoPin, 255);

  /*
   * This block of code is just for troubleshooting if there are issues 
   * To take advantage of it hook the arduino up to the computer using a 
   * usb A male to B male cable.
   * Then open Arduino ide
   * Then click on "Tools" -> "Serial Monitor"
   * Then make sure that 9600 baud is selected in the bottom right. 
   */
  Serial.println("");
  Serial.println("");
  Serial.println("Movement: Forward");
  Serial.println("");
  Serial.println("");
  delay(1);  
}
//Function to call to make car go backward
void backward(){
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, HIGH);
  analogWrite(pwmMotorOnePin,255); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin,HIGH); 
  analogWrite(pwmMotorTwoPin, 255);

  /*
   * This block of code is just for troubleshooting if there are issues 
   * To take advantage of it hook the arduino up to the computer using a 
   * usb A male to B male cable.
   * Then open Arduino ide
   * Then click on "Tools" -> "Serial Monitor"
   * Then make sure that 9600 baud is selected in the bottom right. 
   */
  Serial.println("");
  Serial.println("");
  Serial.println("Movement: Backward");
  Serial.println("");
  Serial.println("");
  delay(1);  
}
//function to call to make car go left
void left(){
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, LOW);
  analogWrite(pwmMotorOnePin,255); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin,HIGH); 
  analogWrite(pwmMotorTwoPin, 255);


  /*
   * This block of code is just for troubleshooting if there are issues 
   * To take advantage of it hook the arduino up to the computer using a 
   * usb A male to B male cable.
   * Then open Arduino ide
   * Then click on "Tools" -> "Serial Monitor"
   * Then make sure that 9600 baud is selected in the bottom right. 
   */
  Serial.println("");
  Serial.println("");
  Serial.println("Movement: Left");
  Serial.println("");
  Serial.println("");
  delay(1);  
}
//function to call to make car go right
void right(){
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, HIGH);
  analogWrite(pwmMotorOnePin,255); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin,LOW); 
  analogWrite(pwmMotorTwoPin, 255);

  /*
   * This block of code is just for troubleshooting if there are issues 
   * To take advantage of it hook the arduino up to the computer using a 
   * usb A male to B male cable.
   * Then open Arduino ide
   * Then click on "Tools" -> "Serial Monitor"
   * Then make sure that 9600 baud is selected in the bottom right. 
   */
  Serial.println("");
  Serial.println("");
  Serial.println("Movement: Right");
  Serial.println("");
  Serial.println("");
  delay(1);  
}

void leftForward(){
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, LOW);
  analogWrite(pwmMotorOnePin,255); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin,LOW); 
  analogWrite(pwmMotorTwoPin, 128);

  /*
   * This block of code is just for troubleshooting if there are issues 
   * To take advantage of it hook the arduino up to the computer using a 
   * usb A male to B male cable.
   * Then open Arduino ide
   * Then click on "Tools" -> "Serial Monitor"
   * Then make sure that 9600 baud is selected in the bottom right. 
   */
  Serial.println("");
  Serial.println("");
  Serial.println("Movement: Left forward");
  Serial.println("");
  Serial.println("");
  delay(1);  
}

void rightForward(){
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, LOW);
  analogWrite(pwmMotorOnePin,128); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin,LOW); 
  analogWrite(pwmMotorTwoPin, 255);

  /*
   * This block of code is just for troubleshooting if there are issues 
   * To take advantage of it hook the arduino up to the computer using a 
   * usb A male to B male cable.
   * Then open Arduino ide
   * Then click on "Tools" -> "Serial Monitor"
   * Then make sure that 9600 baud is selected in the bottom right. 
   */
  Serial.println("");
  Serial.println("");
  Serial.println("Movement: Right forward");
  Serial.println("");
  Serial.println("");
  delay(1);  
}

void leftBackward(){
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, HIGH);
  analogWrite(pwmMotorOnePin,255); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin,HIGH); 
  analogWrite(pwmMotorTwoPin, 128);

  /*
   * This block of code is just for troubleshooting if there are issues 
   * To take advantage of it hook the arduino up to the computer using a 
   * usb A male to B male cable.
   * Then open Arduino ide
   * Then click on "Tools" -> "Serial Monitor"
   * Then make sure that 9600 baud is selected in the bottom right. 
   */
  Serial.println("");
  Serial.println("");
  Serial.println("Movement: Left backward");
  Serial.println("");
  Serial.println("");
  delay(1);  
}

void rightBackward(){
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, HIGH);
  analogWrite(pwmMotorOnePin,128); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin,HIGH); 
  analogWrite(pwmMotorTwoPin, 255);

  /*
   * This block of code is just for troubleshooting if there are issues 
   * To take advantage of it hook the arduino up to the computer using a 
   * usb A male to B male cable.
   * Then open Arduino ide
   * Then click on "Tools" -> "Serial Monitor"
   * Then make sure that 9600 baud is selected in the bottom right. 
   */
  Serial.println("");
  Serial.println("");
  Serial.println("Movement: Right backward");
  Serial.println("");
  Serial.println("");
  delay(1);  
}


//Function to call  to make the car stationary (Brake)
void brake(){
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, LOW);
  analogWrite(pwmMotorOnePin,0); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin,LOW); 
  analogWrite(pwmMotorTwoPin, 0);

    /*
   * This block of code is just for troubleshooting if there are issues 
   * To take advantage of it hook the arduino up to the computer using a 
   * usb A male to B male cable.
   * Then open Arduino ide
   * Then click on "Tools" -> "Serial Monitor"
   * Then make sure that 9600 baud is selected in the bottom right. 
   */
  Serial.println("");
  Serial.println("");
  Serial.println("Movement: Brake");
  Serial.println("");
  Serial.println("");
  delay(1);  
}

void emergencyBrakeButton(){
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, LOW);
  analogWrite(pwmMotorOnePin,0); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin,LOW); 
  analogWrite(pwmMotorTwoPin, 0);

    /*
   * This block of code is just for troubleshooting if there are issues 
   * To take advantage of it hook the arduino up to the computer using a 
   * usb A male to B male cable.
   * Then open Arduino ide
   * Then click on "Tools" -> "Serial Monitor"
   * Then make sure that 9600 baud is selected in the bottom right. 
   */
  Serial.println("");
  Serial.println("");
  Serial.println("Movement: Emergency Brake engaged by button");
  Serial.println("");
  Serial.println("");
  delay(1);  
}

void emergencyBrakeBluetooth(){
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, LOW);
  analogWrite(pwmMotorOnePin,0); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin,LOW); 
  analogWrite(pwmMotorTwoPin, 0);

    /*
   * This block of code is just for troubleshooting if there are issues 
   * To take advantage of it hook the arduino up to the computer using a 
   * usb A male to B male cable.
   * Then open Arduino ide
   * Then click on "Tools" -> "Serial Monitor"
   * Then make sure that 9600 baud is selected in the bottom right. 
   */
  Serial.println("");
  Serial.println("");
  Serial.println("Movement: Emergency Brake engaged by button");
  Serial.println("");
  Serial.println("");
  delay(1);  
}


