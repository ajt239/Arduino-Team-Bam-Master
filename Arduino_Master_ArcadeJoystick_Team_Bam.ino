/*
 * Author: Alex Thomas 
 * 
 * This sketch is not mean to work with push buttons or an Xbox 36o controller 
 * 
 * This sketch is was written for CS486C Team B.A.M. Capstone project. 
 * It is meant to give a 12 volt ride-on-toy car the ability to be controlled through a
 * joystick, an arduino uno, and two md10c rev 3 motor drivers. 
 * The joystick needs to be a button or switched based joystick. This means joysticks with potentiometers
 * will not work. 
 * For instructions on how to set up this code with the appropriate hardware and in the car please visit:
 * insert website here 
 * 
 * For other queries you can reach me at ajt239@nau.edu 
 * This is a student email. I am not sure how long it will stay active after I graduate. 
 */

#include <SoftwareSerial.h>

//define pins and create serial connection for bluetooth adapter 
//Pin 7 for TXD
//pin 8 for RXD 
SoftwareSerial bluetoothSerial(7,8); 



//setting Pin numberss for motor one.
//const int because it is not meant to change.  
const int motorOneDirectionPin = 12;
const int pwmMotorOnePin = 9; 

//setting pin numbers for motor 2 
//const int because it is not meant to change.
const int motorTwoDirectionPin = 10;
const int pwmMotorTwoPin = 11; 

//setting pin numbers for joy stick.
//const int because it is not meant to change. 
const int joystickForwardPin = 2; 
const int joystickBackwardPin = 3; 
const int joystickRightPin = 4; 
const int joystickLeftPin = 5; 

//variables to help with controlling motor speed once bluetooth is integratsed 
int motorOneSpeed;
int motorTwoSpeed;

int topSpeed = 240;

//Variable to count how many times the emergency brake button is pushed.
int emergencyBrakeCounter;

//This function is required by the arduino. 
//It is for setting up the pins on the arduino and for starting serial connections. 
void setup() {
  //begin Serial for troubleshooting
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
  pinMode(joystickForwardPin, INPUT);
  pinMode(joystickBackwardPin, INPUT); 
  pinMode(joystickRightPin, INPUT); 
  pinMode(joystickLeftPin, INPUT);
  Serial.println("Joystick pins set to INPUT."); 
}

//This function is required by the arduino
//It is where the code goes that needs to loop multiple times. 
void loop() {

  //setup variables that will read the state of the input pins. 
  int forwardPinState = digitalRead(joystickForwardPin);
  int backwardPinState = digitalRead(joystickBackwardPin); 
  int rightPinState = digitalRead(joystickRightPin); 
  int leftPinState = digitalRead(joystickLeftPin); 

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
  delay(1);
  
  
  int bluetoothData;
  if (bluetoothSerial.available()) {
    bluetoothData = bluetoothSerial.read();  
    Serial.println("Input from Bluetooth: ");
    if(bluetoothData == 10){
      emergencyBrakeCounter++;
    }else if(emergencyBrakeCounter%2 == 1){
      emergencyBrake();
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
    if(emergencyBrakeCounter%2 ==1){
      emergencyBrake();
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
      if(forwardPinState == 0){
        rightForward();
      }else if (backwardPinState == 0){
        rightBackward();
      }else{
        right();
      }
    }else if(leftPinState == 0){
      if(forwardPinState == 0){
        leftForward(); 
      }else if (backwardPinState == 0){
        leftBackward(); 
      }else{
        left();
      }
    }else{
      brake();
    }
  }
}

//Function to gradually accelerate Motors
//Function to call to make car go forward
void forward(){
  if(motorOneSpeed != topSpeed && motorTwoSpeed != topSpeed){
    for(int i = 100; i <= topSpeed; i += 20){
      motorOneSpeed = i; 
      motorTwoSpeed = i; 
      digitalWrite(motorOneDirectionPin, HIGH);
      analogWrite(pwmMotorOnePin,motorOneSpeed);
      digitalWrite(motorTwoDirectionPin, HIGH); 
      analogWrite(pwmMotorTwoPin, motorTwoSpeed);
      Serial.print("Motor 1 speed: ");
      Serial.println(motorOneSpeed);
      Serial.print("Motor 2 speed: ");
      Serial.println(motorTwoSpeed);
      delay(100);
      if(digitalRead(joystickForwardPin) == 1){
        break;  
      }
    }
  }else{
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, HIGH);
  analogWrite(pwmMotorOnePin,motorOneSpeed); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin, HIGH); 
  analogWrite(pwmMotorTwoPin, motorTwoSpeed);

  Serial.println(motorOneSpeed);
  Serial.println(motorTwoSpeed); 

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
}

//Function to call to make car go backward
void backward(){
   if(motorOneSpeed != topSpeed && motorTwoSpeed != topSpeed){
    for(int i = 100; i <= topSpeed; i += 20){
      motorOneSpeed = i; 
      motorTwoSpeed = i; 
      digitalWrite(motorOneDirectionPin, LOW);
      analogWrite(pwmMotorOnePin,motorOneSpeed);
      digitalWrite(motorTwoDirectionPin, LOW); 
      analogWrite(pwmMotorTwoPin, motorTwoSpeed);
      Serial.print("Motor 1 speed: ");
      Serial.println(motorOneSpeed);
      Serial.print("Motor 2 speed: ");
      Serial.println(motorTwoSpeed);
      delay(100);
      if(digitalRead(joystickBackwardPin) == 1){
        break;  
      }
    }
  }else{
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, LOW);
  analogWrite(pwmMotorOnePin,motorOneSpeed); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin, LOW); 
  analogWrite(pwmMotorTwoPin, motorTwoSpeed);

  Serial.println(motorOneSpeed);
  Serial.println(motorTwoSpeed); 

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
  
}
//function to call to make car go left
void left(){
  if(motorOneSpeed != topSpeed && motorTwoSpeed != topSpeed){
    for(int i = 100; i <= topSpeed; i += 20){
      motorOneSpeed = i; 
      motorTwoSpeed = i; 
      digitalWrite(motorOneDirectionPin, HIGH);
      analogWrite(pwmMotorOnePin,motorOneSpeed);
      digitalWrite(motorTwoDirectionPin, LOW); 
      analogWrite(pwmMotorTwoPin, motorTwoSpeed);
      Serial.print("Motor 1 speed: ");
      Serial.println(motorOneSpeed);
      Serial.print("Motor 2 speed: ");
      Serial.println(motorTwoSpeed);
      delay(100);
      if(digitalRead(joystickLeftPin) == 1){
        break;  
      }
    }
  }else{
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, HIGH);
  analogWrite(pwmMotorOnePin,motorOneSpeed); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin, LOW); 
  analogWrite(pwmMotorTwoPin, motorTwoSpeed);

  Serial.println(motorOneSpeed);
  Serial.println(motorTwoSpeed); 

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
}

//function to call to make car go right
void right(){
   if(motorOneSpeed != topSpeed && motorTwoSpeed != topSpeed){
    for(int i = 100; i <= topSpeed; i += 20){
      motorOneSpeed = i; 
      motorTwoSpeed = i; 
      digitalWrite(motorOneDirectionPin, LOW);
      analogWrite(pwmMotorOnePin,motorOneSpeed);
      digitalWrite(motorTwoDirectionPin, HIGH); 
      analogWrite(pwmMotorTwoPin, motorTwoSpeed);
      Serial.print("Motor 1 speed: ");
      Serial.println(motorOneSpeed);
      Serial.print("Motor 2 speed: ");
      Serial.println(motorTwoSpeed);
      delay(100);
      if(digitalRead(joystickRightPin) == 1){
        break;  
      }
    }
  }else{
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, LOW);
  analogWrite(pwmMotorOnePin,motorOneSpeed); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin, HIGH); 
  analogWrite(pwmMotorTwoPin, motorTwoSpeed);

  Serial.println(motorOneSpeed);
  Serial.println(motorTwoSpeed); 

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
}

void leftForward(){
   if(motorOneSpeed != topSpeed && motorTwoSpeed != topSpeed){
    for(int i = 100; i <= topSpeed; i += 20){
      motorOneSpeed = i; 
      motorTwoSpeed = i; 
      digitalWrite(motorOneDirectionPin, HIGH);
      analogWrite(pwmMotorOnePin,motorOneSpeed);
      digitalWrite(motorTwoDirectionPin, HIGH); 
      analogWrite(pwmMotorTwoPin, (motorTwoSpeed/2));
      Serial.print("Motor 1 speed: ");
      Serial.println(motorOneSpeed);
      Serial.print("Motor 2 speed: ");
      Serial.println(motorTwoSpeed);
      delay(100);
      if(digitalRead(joystickForwardPin) == 1){
        if(digitalRead(joystickLeftPin) == 1){
          break; 
        }
      }else if(digitalRead(joystickLeftPin) == 1){
        if(digitalRead(joystickForwardPin) == 1){
          break; 
        }
      }
    }
  }else{
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, HIGH);
  analogWrite(pwmMotorOnePin,motorOneSpeed); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin, HIGH); 
  analogWrite(pwmMotorTwoPin, (motorTwoSpeed/2));

  Serial.println(motorOneSpeed);
  Serial.println(motorTwoSpeed); 

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
  Serial.println("Movement: Left Forward");
  Serial.println("");
  Serial.println("");
  
  delay(1);
  }  
}

void rightForward(){
  if(motorOneSpeed != topSpeed && motorTwoSpeed != topSpeed){
    for(int i = 100; i <= topSpeed; i += 20){
      motorOneSpeed = i; 
      motorTwoSpeed = i; 
      digitalWrite(motorOneDirectionPin, HIGH);
      analogWrite(pwmMotorOnePin,(motorOneSpeed/2));
      digitalWrite(motorTwoDirectionPin, HIGH); 
      analogWrite(pwmMotorTwoPin, motorTwoSpeed);
      Serial.print("Motor 1 speed: ");
      Serial.println(motorOneSpeed);
      Serial.print("Motor 2 speed: ");
      Serial.println(motorTwoSpeed);
      delay(100);
      if(digitalRead(joystickForwardPin) == 1){
        if(digitalRead(joystickRightPin) == 1){
          break; 
        }
      }else if(digitalRead(joystickRightPin) == 1){
        if(digitalRead(joystickForwardPin) == 1){
          break; 
        }
      }
    }
  }else{
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, HIGH);
  analogWrite(pwmMotorOnePin,(motorOneSpeed/2)); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin, HIGH); 
  analogWrite(pwmMotorTwoPin, motorTwoSpeed);

  Serial.println(motorOneSpeed);
  Serial.println(motorTwoSpeed); 

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
  Serial.println("Movement: Right Forward");
  Serial.println("");
  Serial.println("");
  
  delay(1);
  }  
}

void leftBackward(){
    if(motorOneSpeed != topSpeed && motorTwoSpeed != topSpeed){
    for(int i = 100; i <= topSpeed; i += 20){
      motorOneSpeed = i; 
      motorTwoSpeed = i; 
      digitalWrite(motorOneDirectionPin, LOW);
      analogWrite(pwmMotorOnePin,motorOneSpeed);
      digitalWrite(motorTwoDirectionPin, LOW); 
      analogWrite(pwmMotorTwoPin, (motorTwoSpeed/2));
      Serial.print("Motor 1 speed: ");
      Serial.println(motorOneSpeed);
      Serial.print("Motor 2 speed: ");
      Serial.println(motorTwoSpeed);
      delay(100);
      if(digitalRead(joystickBackwardPin) == 1){
        if(digitalRead(joystickLeftPin) == 1){
          break; 
        }
      }else if(digitalRead(joystickLeftPin) == 1){
        if(digitalRead(joystickBackwardPin) == 1){
          break; 
        }
      }
    }
  }else{
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, LOW);
  analogWrite(pwmMotorOnePin,motorOneSpeed); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin, LOW); 
  analogWrite(pwmMotorTwoPin, (motorTwoSpeed/2));

  Serial.println(motorOneSpeed);
  Serial.println(motorTwoSpeed); 

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
  Serial.println("Movement: Left Backward");
  Serial.println("");
  Serial.println("");
  
  delay(1);
  }  
}

void rightBackward(){
    if(motorOneSpeed != topSpeed && motorTwoSpeed != topSpeed){
    for(int i = 100; i <= topSpeed; i += 20){
      motorOneSpeed = i; 
      motorTwoSpeed = i; 
      digitalWrite(motorOneDirectionPin, LOW);
      analogWrite(pwmMotorOnePin,(motorOneSpeed/2));
      digitalWrite(motorTwoDirectionPin, LOW); 
      analogWrite(pwmMotorTwoPin, motorTwoSpeed);
      Serial.print("Motor 1 speed: ");
      Serial.println(motorOneSpeed);
      Serial.print("Motor 2 speed: ");
      Serial.println(motorTwoSpeed);
      delay(100);
      if(digitalRead(joystickBackwardPin) == 1){
        if(digitalRead(joystickRightPin) == 1){
          break; 
        }
      }else if(digitalRead(joystickRightPin) == 1){
        if(digitalRead(joystickBackwardPin) == 1){
          break; 
        }
      }
    }
  }else{
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, LOW);
  analogWrite(pwmMotorOnePin,(motorOneSpeed/2)); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin, LOW); 
  analogWrite(pwmMotorTwoPin, motorTwoSpeed);

  Serial.println(motorOneSpeed);
  Serial.println(motorTwoSpeed); 

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
  Serial.println("Movement: Right Backward");
  Serial.println("");
  Serial.println("");
  
  delay(1);
  }  
}

//Function to call  to make the car stationary (Brake)
void brake(){
  motorOneSpeed = 0;
  motorTwoSpeed = 0;
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, LOW);
  analogWrite(pwmMotorOnePin,motorOneSpeed); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin,LOW); 
  analogWrite(pwmMotorTwoPin, motorTwoSpeed);

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

void emergencyBrake(){
  motorOneSpeed = 0;
  motorTwoSpeed = 0;
  //set motor one direction and pwm pulse rate 
  digitalWrite(motorOneDirectionPin, LOW);
  analogWrite(pwmMotorOnePin, motorOneSpeed); 

  //set motor two direction and pwm pulse rate 
  digitalWrite(motorTwoDirectionPin,motorTwoSpeed); 
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
  Serial.println("Movement: Emergency Brake engaged by bluetooth");
  Serial.println("");
  Serial.println("");
  delay(1);  
}
