#include <SoftwareSerial.h>

//-----------------START BLUETOOTH SETUP-------------------------------------------------------

//define pins and create serial connection for bluetooth adapter 
//Pin 7 for TXD
//pin 8 for RXD 
SoftwareSerial bluetoothSerial(7,8);

//Setting up pin to check the state of if a device is connected to the bluetooth module.
//This is used to stop the car is a device is connected and then loses connection. 
const int bluetoothStatePin = 6;  

//Variable to hold data from bluetooth 
int bluetoothData = -1;

//check bluetooth State
int bluetoothState;

//-----------------END BLUETOOTH SETUP---------------------------------------------------------

//-----------------START MOTOR PIN SETUP-------------------------------------------------------

//setting Pin numberss for motor one.
//const int because it is not meant to change.  
const int motorOneDirectionPin = 10;
const int pwmMotorOnePin = 9; 

//setting pin numbers for motor 2 
//const int because it is not meant to change.
const int motorTwoDirectionPin = 12;
const int pwmMotorTwoPin = 11; 

//-----------------END MOTOR PIN SETUP---------------------------------------------------------

//-----------------START JOYSTICK SETUP--------------------------------------------------------

//setting pin numbers for joy stick.
//const int because it is not meant to change. 
const int joystickForwardPin = 2; 
const int joystickBackwardPin = 3; 
const int joystickRightPin = 4; 
const int joystickLeftPin = 5; 

//-----------------END JOYSTICK SETUP----------------------------------------------------------

//-----------------START SPEED CONTROL SETUP---------------------------------------------------

//variables that will help control the speed of the car.

int zeroPercent = 0; 
int twentyFivePercent = 139;
int fiftyPercent = 178;
int seventyFivePercent = 216;
int oneHundredPercent = 255; 

//The default for the top speed of the car is oneHundredPercent 
int topSpeed = oneHundredPercent; 

//define variable that can be used for graduale acceleration 
int gradualAcceleration;
int i; 
//-----------------END SPEED CONTROL SETUP-----------------------------------------------------

//-----------------START Parental CONTROL SETUP------------------------------------------------

//Setup parental control boolean 
boolean parentalControl = false;

//Variable to count how many times the emergency brake button is pushed.
int emergencyBrakeCounter;

//-----------------END SPEED CONTROL SETUP-----------------------------------------------------


void setup() {
  //begin bluetooth Serial Connection 
  bluetoothSerial.begin(9600);
  //Create Serial Connection to Computer for Troubleshooting 
  Serial.begin(9600);
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

  Serial.print("Parental Control Value: "); 
  Serial.println(parentalControl); 
}

void loop() {
  
  //setup variables that will read the state of the input pins. 
  int forwardPinState = digitalRead(joystickForwardPin);
  int backwardPinState = digitalRead(joystickBackwardPin); 
  int rightPinState = digitalRead(joystickRightPin); 
  int leftPinState = digitalRead(joystickLeftPin);
  Serial.println(topSpeed); 
  /*
   * This block of code is just for troubleshooting if there are issues 
   * To take advantage of it hook the arduino up to the computer using a 
   * usb A male to B male cable.
   * Then open Arduino ide
   * Then click on "Tools" -> "Serial Monitor"
   * Then make sure that 9600 baud is selected in the bottom right. 
   * Send value of pin states for torubleshooting and testing 
   * first print line is to create space between setup when loop first runs. 
   */
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
  Serial.print("Bluetooth data integer: ");
  Serial.println(bluetoothData); 
  
  //read in bluetoothState pin and set value to bluetooth State 
  bluetoothState = digitalRead(bluetoothStatePin);
  //Print our bluetooth State to serial connection for trouble shooting 
  Serial.print("Bluetooth state: ");
  Serial.println(bluetoothState);

  //read in data from bluetoothSerial connection and then set it to bluetoothData 
  bluetoothData = bluetoothSerial.read(); 

  /*
   * checks to see if the car has a blue tooth connection
   * If a device is connected through bluetooth the state will be equal 1
   * if there is no device connected the state will be 0 
   * We set parental control to true once a devices has been connected to the car 
   */
  if (bluetoothState == 1){
    parentalControl = true; 
  }

  /* 
   *  if a device has been connected to the car the bluetoothState will be set to 1 and
   *  the parental control will be set to true. So if the state goes to zero and parent control 
   *  is true stop the car because the parents device has been disconnected. 
   *  
   *  else the car can be used as normal
   */
  if (bluetoothState == 0 && parentalControl == true) {
    bluetoothBrake(); 
  }else {
    if(bluetoothData == 10){
      emergencyBrakeCounter++;
      Serial.println(""); 
      Serial.print("EmergencyBrakeCounter: ");
      Serial.println(emergencyBrakeCounter); 
    }else if (bluetoothData == 0){
      bluetoothBrake();
    }else if (bluetoothData == 1){
      bluetoothForward();
    }else if (bluetoothData == 2){
      bluetoothBackward();
    }else if(bluetoothData == 3){
      bluetoothLeft();
    }else if(bluetoothData ==4){
      bluetoothRight();
    }else if(bluetoothData == 5){
      bluetoothLeftForward();
    }else if(bluetoothData == 6){
      bluetoothRightForward(); 
    }else if (bluetoothData == 7){
      bluetoothLeftBackward();
    }else if (bluetoothData == 8){
      bluetoothRightBackward();
    }else if(bluetoothData == 9){
      topSpeed = zeroPercent; 
    }else if (bluetoothData == 25){ 
      topSpeed = twentyFivePercent; 
    }else if (bluetoothData == 50){
      topSpeed = fiftyPercent; 
    }else if (bluetoothData == 75){
      topSpeed = seventyFivePercent; 
    }else if (bluetoothData == 100){
      topSpeed = oneHundredPercent; 
    }//else if(emergencyBrakeCounter%2 == 0){
      //Serial.println("#####BLUETOOTH OVERRIDE NOT ENGAGED");
    //}
    else{
      if(emergencyBrakeCounter%2 ==1){
        emergencyOverride();
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
      }
    }
  }//end else statement that runs if parent device is connected and if parent device was 
   //never connected to the car 
}



void bluetoothForward(){
   if(gradualAcceleration != topSpeed){
    for(i = 100; i <= topSpeed; i++){
      gradualAcceleration = i; 
      digitalWrite(motorOneDirectionPin, HIGH);
      analogWrite(pwmMotorOnePin, gradualAcceleration);
      digitalWrite(motorTwoDirectionPin, HIGH);
      analogWrite(pwmMotorTwoPin, gradualAcceleration);
      Serial.print("Gradual Acceleration speed: "); 
      Serial.println(gradualAcceleration); 
      delay(1); 
      if (bluetoothSerial.available()){
        bluetoothBrake();
        break;
      }
    }
  }else{
    //original code 
    //set motor one direction and pwm pulse rate 
    digitalWrite(motorOneDirectionPin, HIGH);
    analogWrite(pwmMotorOnePin,topSpeed); 
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, HIGH); 
    analogWrite(pwmMotorTwoPin, topSpeed);
  
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
    Serial.println("Movement: Bluetooth Forward");
    Serial.println("");
    Serial.println("");
    
    delay(1);
    //end original code 
  }
}



void bluetoothBackward(){
   if(gradualAcceleration != topSpeed){
    for(i = 100; i <= topSpeed; i++){
      gradualAcceleration = i; 
      digitalWrite(motorOneDirectionPin, LOW);
      analogWrite(pwmMotorOnePin, gradualAcceleration);
      digitalWrite(motorTwoDirectionPin, LOW);
      analogWrite(pwmMotorTwoPin, gradualAcceleration);
      Serial.print("Gradual Acceleration speed: "); 
      Serial.println(gradualAcceleration); 
      delay(1); 
      if (bluetoothSerial.available()){
        bluetoothBrake();
        break;
      }
    }
  }else{
    //original code 
    //set motor one direction and pwm pulse rate 
    digitalWrite(motorOneDirectionPin, LOW);
    analogWrite(pwmMotorOnePin,topSpeed); 
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, LOW); 
    analogWrite(pwmMotorTwoPin, topSpeed);
  
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
    Serial.println("Movement: Bluetooth Backward");
    Serial.println("");
    Serial.println("");
    
    delay(1);
    //end original code 
  }
}



void bluetoothLeft(){
   if(gradualAcceleration != topSpeed){
    for(i = 100; i <= topSpeed; i++){
      gradualAcceleration = i; 
      digitalWrite(motorOneDirectionPin, LOW);
      analogWrite(pwmMotorOnePin, gradualAcceleration);
      digitalWrite(motorTwoDirectionPin, HIGH);
      analogWrite(pwmMotorTwoPin, gradualAcceleration);
      Serial.print("Gradual Acceleration speed: "); 
      Serial.println(gradualAcceleration); 
      delay(1); 
      if (bluetoothSerial.available()){
        bluetoothBrake();
        break;
      }
    }
  }else{
    //original code 
     //set motor one direction and pwm pulse rate 
    digitalWrite(motorOneDirectionPin, LOW);
    analogWrite(pwmMotorOnePin,topSpeed); 
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, HIGH); 
    analogWrite(pwmMotorTwoPin, topSpeed);
  
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
    Serial.println("Movement: Bluetooth Left");
    Serial.println("");
    Serial.println("");
    
    delay(1);
    //end original code
  }
}



void bluetoothRight(){
   if(gradualAcceleration != topSpeed){
    for(i = 100; i <= topSpeed; i++){
      gradualAcceleration = i; 
      digitalWrite(motorOneDirectionPin, HIGH);
      analogWrite(pwmMotorOnePin, gradualAcceleration);
      digitalWrite(motorTwoDirectionPin, LOW);
      analogWrite(pwmMotorTwoPin, gradualAcceleration);
      Serial.print("Gradual Acceleration speed: "); 
      Serial.println(gradualAcceleration); 
      delay(1); 
      if (bluetoothSerial.available()){
        bluetoothBrake();
        break;
      }
    }
  }else{
    //original code 
    //set motor one direction and pwm pulse rate 
    digitalWrite(motorOneDirectionPin, HIGH);
    analogWrite(pwmMotorOnePin,topSpeed); 
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, LOW); 
    analogWrite(pwmMotorTwoPin, topSpeed);
  
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
    Serial.println("Movement: Bluetooth Right");
    Serial.println("");
    Serial.println("");
    
    delay(1);
    //end original code 
  }
}



void bluetoothLeftForward(){
  if(gradualAcceleration != topSpeed){
    for(i = 100; i <= topSpeed; i++){
      gradualAcceleration = i; 
      digitalWrite(motorOneDirectionPin, HIGH);
      analogWrite(pwmMotorOnePin, (gradualAcceleration/2));
      digitalWrite(motorTwoDirectionPin, HIGH);
      analogWrite(pwmMotorTwoPin, gradualAcceleration);
      Serial.print("Gradual Acceleration speed: "); 
      Serial.println(gradualAcceleration); 
      delay(1); 
      if (bluetoothSerial.available()){
        bluetoothBrake();
        break;
      }
    }
  }else{
    //original code 
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
    Serial.println("Movement: bluetooth Left Forward Function (bluetoothLeftForward())");
    Serial.println("");
    Serial.println("");
  
    /*
     * This block of code is the actual functionanilty for the bluetoothLeftForward
     * funcion. It lowers the the motor one motor by half of the topSpeed to achieve a left 
     * hand turn. The right hand motor then is kept at topSpeed to push the car left. 
     */
    //set motor one direction and pwm pulse rate
    digitalWrite(motorOneDirectionPin, HIGH);
    analogWrite(pwmMotorOnePin,(topSpeed/2));
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, HIGH); 
    analogWrite(pwmMotorTwoPin, topSpeed);
    //end original code 
  }
}



void bluetoothRightForward(){
  if(gradualAcceleration != topSpeed){
    for(i = 100; i <= topSpeed; i++){
      gradualAcceleration = i; 
      digitalWrite(motorOneDirectionPin, HIGH);
      analogWrite(pwmMotorOnePin, gradualAcceleration);
      digitalWrite(motorTwoDirectionPin, HIGH);
      analogWrite(pwmMotorTwoPin, (gradualAcceleration/2));
      Serial.print("Gradual Acceleration speed: "); 
      Serial.println(gradualAcceleration); 
      delay(1); 
      if (bluetoothSerial.available()){
        bluetoothBrake();
        break;
      }
    }
  }else{
    //original code 
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
    Serial.println("Movement: bluetooth Right Forward Function (bluetoothRightForward())");
    Serial.println("");
    Serial.println("");
  
    /*
     * This block of code is the actual functionanilty for the bluetoothRightForward
     * funcion. It lowers the the motor two motor by half of the topSpeed to achieve a right 
     * hand turn. The left hand motor then is kept at topSpeed to push the car right. 
     */
    //set motor one direction and pwm pulse rate
    digitalWrite(motorOneDirectionPin, HIGH);
    analogWrite(pwmMotorOnePin,topSpeed);
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, HIGH); 
    analogWrite(pwmMotorTwoPin, (topSpeed/2));
    //end original code 
  }
}



void bluetoothLeftBackward(){
  if(gradualAcceleration != topSpeed){
    for(i = 100; i <= topSpeed; i++){
      gradualAcceleration = i; 
      digitalWrite(motorOneDirectionPin, LOW);
      analogWrite(pwmMotorOnePin, (gradualAcceleration/2));
      digitalWrite(motorTwoDirectionPin, LOW);
      analogWrite(pwmMotorTwoPin, gradualAcceleration);
      Serial.print("Gradual Acceleration speed: "); 
      Serial.println(gradualAcceleration); 
      delay(1); 
      if (bluetoothSerial.available()){
        bluetoothBrake();
        break;
      }
    }
  }else{
     //original code 
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
    Serial.println("Movement: bluetooth Left Backward Function (bluetoothLeftBackward())");
    Serial.println("");
    Serial.println("");
  
    /*
     * This block of code is the actual functionanilty for the bluetoothLeftBackward
     * funcion. It lowers the the motor one motor by half of the topSpeed to achieve a backward left 
     * hand turn. The right hand motor then is kept at topSpeed to push the car left. 
     */
    //set motor one direction and pwm pulse rate
    digitalWrite(motorOneDirectionPin, LOW);
    analogWrite(pwmMotorOnePin,(topSpeed/2));
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, LOW); 
    analogWrite(pwmMotorTwoPin, topSpeed);
    //end original code
  }
}



void bluetoothRightBackward(){
  if(gradualAcceleration != topSpeed){
    for(i = 100; i <= topSpeed; i++){
      gradualAcceleration = i; 
      digitalWrite(motorOneDirectionPin, LOW);
      analogWrite(pwmMotorOnePin, gradualAcceleration);
      digitalWrite(motorTwoDirectionPin, LOW);
      analogWrite(pwmMotorTwoPin, (gradualAcceleration/2));
      Serial.print("Gradual Acceleration speed: "); 
      Serial.println(gradualAcceleration); 
      delay(1); 
      if (bluetoothSerial.available()){
        bluetoothBrake();
        break;
      }
    }
  }else{
    //original code 
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
    Serial.println("Movement: bluetooth Right Backward Function (bluetoothRightBackward())");
    Serial.println("");
    Serial.println("");
  
    /*
     * This block of code is the actual functionanilty for the bluetoothRightBackward
     * funcion. It lowers the the motor two motor by half of the topSpeed to achieve a backward right 
     * hand turn. The left hand motor then is kept at topSpeed to push the car right. 
     */
    //set motor one direction and pwm pulse rate
    digitalWrite(motorOneDirectionPin, LOW);
    analogWrite(pwmMotorOnePin,topSpeed);
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, LOW); 
    analogWrite(pwmMotorTwoPin, (topSpeed/2));
    //end original code 
  }
}



void bluetoothBrake(){
   gradualAcceleration = 100; 
   i = 100;
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
  Serial.println("Movement: Bluetooth Brake Function (bluetoothBrake())");
  Serial.println("");
  Serial.println(""); 

  
  /*
   * This block of code is the actual functionanilty for the bluetoothBrake
   * function. It basically sets the directions to LOW and then sets the pwm 
   * pulse rate to zero which is what actually stops the motors from spinning 
   */
  
  //set motor one direction and pwm pulse rate
  //DigitalWrite controls directions, Analog write PWM 
  digitalWrite(motorOneDirectionPin, LOW); 
  analogWrite(pwmMotorOnePin, zeroPercent); 

  //set motor two direction and pwm pulse rate 
  //DigitalWrite controls directions, Analog write PWM 
  digitalWrite(motorTwoDirectionPin, LOW); 
  analogWrite(pwmMotorTwoPin, zeroPercent); 
}

void emergencyOverride(){
  Serial.println("");
  Serial.println("");
  Serial.println("##########Bluetooth Override Engaged##########");
  Serial.println("");
  Serial.println("");

  if(bluetoothData == 0){
    bluetoothBrake();
  }else if (bluetoothData == 1){
      bluetoothForward();
  }else if (bluetoothData == 2){
     bluetoothBackward();
  }else if(bluetoothData == 3){
      bluetoothLeft();
  }else if(bluetoothData ==4){
      bluetoothRight();
  }else if(bluetoothData == 5){
    bluetoothLeftForward();
  }else if(bluetoothData == 6){
    bluetoothRightForward(); 
  }else if (bluetoothData == 7){
    bluetoothLeftBackward();
  }else if (bluetoothData == 8){
    bluetoothRightBackward();
  }else if(bluetoothData == 9){
    topSpeed = zeroPercent; 
  }else if (bluetoothData == 25){ 
    topSpeed = twentyFivePercent; 
  }else if (bluetoothData == 50){
    topSpeed = fiftyPercent; 
  }else if (bluetoothData == 75){
    topSpeed = seventyFivePercent; 
  }else if (bluetoothData == 100){
    topSpeed = oneHundredPercent; 
  }
}


void forward(){
  if(gradualAcceleration != topSpeed){
    for(i = 100; i <= topSpeed; i++){
      gradualAcceleration = i; 
      digitalWrite(motorOneDirectionPin, HIGH);
      analogWrite(pwmMotorOnePin, gradualAcceleration);
      digitalWrite(motorTwoDirectionPin, HIGH);
      analogWrite(pwmMotorTwoPin, gradualAcceleration);
      Serial.print("Gradual Acceleration speed: "); 
      Serial.println(gradualAcceleration); 
      delay(1); 
      if(digitalRead(joystickForwardPin) == 1){
        brake(); 
        break;
      }else if (bluetoothSerial.available()){
        break;
      }
    }
  }else{
    //start original 
    //set motor one direction and pwm pulse rate 
    digitalWrite(motorOneDirectionPin, HIGH);
    analogWrite(pwmMotorOnePin,topSpeed); 
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, HIGH); 
    analogWrite(pwmMotorTwoPin, topSpeed);
  
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
    if(digitalRead(joystickForwardPin) == 1){
      brake();
      }
    if(bluetoothSerial.available()){
      bluetoothBrake();  
    }//end original 
  }
}



void backward(){
  if(gradualAcceleration != topSpeed){
    for(i = 100; i <= topSpeed; i++){
       gradualAcceleration = i; 
       digitalWrite(motorOneDirectionPin, LOW);
       analogWrite(pwmMotorOnePin,gradualAcceleration);
       digitalWrite(motorTwoDirectionPin, LOW); 
       analogWrite(pwmMotorTwoPin, gradualAcceleration);
       Serial.print("Gradual Acceleration Speed: ");
       Serial.println(gradualAcceleration);
       delay(100);
       if(digitalRead(joystickBackwardPin) == 1){
         brake();
         break;  
       }else if (bluetoothSerial.available()){
        break;
      }
    }
  }else{
    //original Code 
    //set motor one direction and pwm pulse rate 
    digitalWrite(motorOneDirectionPin, LOW);
    analogWrite(pwmMotorOnePin,topSpeed); 
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, LOW); 
    analogWrite(pwmMotorTwoPin, topSpeed);
  
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
    if(digitalRead(joystickBackwardPin) == 1){
      brake(); 
    }
    if(bluetoothSerial.available()){
      bluetoothBrake();  
    }//end original code 
  }//end new else statement 
}



void left(){
   if(gradualAcceleration != topSpeed){
    for(int i = 100; i <= topSpeed; i++){
       gradualAcceleration = i; 
       digitalWrite(motorOneDirectionPin, LOW);
       analogWrite(pwmMotorOnePin,gradualAcceleration);
       digitalWrite(motorTwoDirectionPin, HIGH); 
       analogWrite(pwmMotorTwoPin, gradualAcceleration);
       Serial.print("Gradual Acceleration Speed: ");
       Serial.println(gradualAcceleration);
       delay(100);
       if(digitalRead(joystickLeftPin) == 1){
         brake();
         break;  
       }else if (bluetoothSerial.available()){
        break;
      }
    }
  }else{
    //Original Code  
    //set motor one direction and pwm pulse rate 
    digitalWrite(motorOneDirectionPin, LOW);
    analogWrite(pwmMotorOnePin,topSpeed); 
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, HIGH); 
    analogWrite(pwmMotorTwoPin, topSpeed);
  
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
    if(digitalRead(joystickLeftPin) == 1){
      brake(); 
    }
    if(bluetoothSerial.available()){
      bluetoothBrake();  
    }//end original Code 
  }
}



void right(){
   if(gradualAcceleration != topSpeed){
    for(int i = 100; i <= topSpeed; i++){
       gradualAcceleration = i; 
       digitalWrite(motorOneDirectionPin, LOW);
       analogWrite(pwmMotorOnePin,gradualAcceleration);
       digitalWrite(motorTwoDirectionPin, HIGH); 
       analogWrite(pwmMotorTwoPin, gradualAcceleration);
       Serial.print("Gradual Acceleration Speed: ");
       Serial.println(gradualAcceleration);
       delay(100);
       if(digitalRead(joystickRightPin) == 1){
         break;  
       }else if (bluetoothSerial.available()){
        break;
      }
    }
  }else{
    //original code 
    //set motor one direction and pwm pulse rate 
    digitalWrite(motorOneDirectionPin, HIGH);
    analogWrite(pwmMotorOnePin,topSpeed); 
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, LOW); 
    analogWrite(pwmMotorTwoPin, topSpeed);
  
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
    if(digitalRead(joystickRightPin) == 1){
      brake(); 
    }
    if(bluetoothSerial.available()){
      bluetoothBrake();  
    }//end original code 
  }
}



//Function to call  to make the car stationary (Brake)
void brake(){
  gradualAcceleration = 100;
  i = 100;  
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



void leftForward(){
  if(gradualAcceleration != topSpeed){
    for(int i = 100; i <= topSpeed; i++){
      gradualAcceleration = i; 
      digitalWrite(motorOneDirectionPin, HIGH);
      analogWrite(pwmMotorOnePin, (gradualAcceleration/2));
      digitalWrite(motorTwoDirectionPin, HIGH);
      analogWrite(pwmMotorTwoPin, gradualAcceleration); 
      delay(100); 
      if(digitalRead(joystickForwardPin) == 1){
        if(digitalRead(joystickLeftPin) == 1){
          brake();
          break; 
        }
      }else if(digitalRead(joystickLeftPin) == 1){
        if(digitalRead(joystickForwardPin) == 1){
          brake();
          break; 
        }
      }else if (bluetoothSerial.available()){
        break;
      }
    }
  }else{
    //original code 
    //set motor one direction and pwm pulse rate 
    digitalWrite(motorOneDirectionPin, HIGH);
    analogWrite(pwmMotorOnePin,(topSpeed/2)); 
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, HIGH); 
    analogWrite(pwmMotorTwoPin, topSpeed);
  
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
    if(digitalRead(joystickLeftPin) == 1){
      if(digitalRead(joystickForwardPin) == 1){
        brake();
      }  
    }
    if(bluetoothSerial.available()){
      bluetoothBrake();  
    }//end original code
  } 
}



void rightForward(){
  if(gradualAcceleration != topSpeed){
    for(int i = 100; i <= topSpeed; i++){
      gradualAcceleration = i; 
      digitalWrite(motorOneDirectionPin, HIGH);
      analogWrite(pwmMotorOnePin, gradualAcceleration);
      digitalWrite(motorTwoDirectionPin, HIGH);
      analogWrite(pwmMotorTwoPin, (gradualAcceleration/2)); 
      delay(100); 
      if(digitalRead(joystickForwardPin) == 1){
        if(digitalRead(joystickRightPin) == 1){
          brake();
          break; 
        }
      }else if(digitalRead(joystickRightPin) == 1){
        if(digitalRead(joystickForwardPin) == 1){
          brake();
          break; 
        }
      }else if (bluetoothSerial.available()){
        break;
      }
    }
  }else{
    //original code 
    //set motor one direction and pwm pulse rate 
    digitalWrite(motorOneDirectionPin, HIGH);
    analogWrite(pwmMotorOnePin,topSpeed); 
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, HIGH); 
    analogWrite(pwmMotorTwoPin, (topSpeed/2));
  
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
    if(digitalRead(joystickRightPin) == 1){
      if(digitalRead(joystickForwardPin) == 1){
        brake();
      }  
    }
    if(bluetoothSerial.available()){
      bluetoothBrake();  
    }//endoriginal code 
  }
}



void leftBackward(){
  if(gradualAcceleration != topSpeed){
    for(int i = 100; i <= topSpeed; i++){
      gradualAcceleration = i; 
      digitalWrite(motorOneDirectionPin, LOW);
      analogWrite(pwmMotorOnePin, (gradualAcceleration/2));
      digitalWrite(motorTwoDirectionPin, LOW);
      analogWrite(pwmMotorTwoPin, gradualAcceleration); 
      delay(100); 
      if(digitalRead(joystickBackwardPin) == 1){
        if(digitalRead(joystickLeftPin) == 1){
          brake(); 
          break; 
        }
      }else if(digitalRead(joystickLeftPin) == 1){
        if(digitalRead(joystickBackwardPin) == 1){
          brake(); 
          break; 
        }
      }else if (bluetoothSerial.available()){
        break;
      }
    }
  }else{
    //original Code
    //set motor one direction and pwm pulse rate 
    digitalWrite(motorOneDirectionPin, LOW);
    analogWrite(pwmMotorOnePin,(topSpeed/2)); 
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, LOW); 
    analogWrite(pwmMotorTwoPin, topSpeed);
  
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
    if(digitalRead(joystickLeftPin) == 1){
      if(digitalRead(joystickBackwardPin) == 1){
        brake();
      }  
    }
    if(bluetoothSerial.available()){
      bluetoothBrake();  
    }//end original code 
  }
}



void rightBackward(){
  if(gradualAcceleration != topSpeed){
    for(int i = 100; i <= topSpeed; i++){
      gradualAcceleration = i; 
      digitalWrite(motorOneDirectionPin, LOW);
      analogWrite(pwmMotorOnePin, gradualAcceleration);
      digitalWrite(motorTwoDirectionPin, LOW);
      analogWrite(pwmMotorTwoPin, (gradualAcceleration/2)); 
      delay(100); 
      if(digitalRead(joystickBackwardPin) == 1){
        if(digitalRead(joystickRightPin) == 1){
          brake(); 
          break; 
        }
      }else if(digitalRead(joystickRightPin) == 1){
        if(digitalRead(joystickBackwardPin) == 1){
          brake(); 
          break; 
        }
      }else if (bluetoothSerial.available()){
        break;
      }
    }
  }else{
    //start original code 
    //set motor one direction and pwm pulse rate 
    digitalWrite(motorOneDirectionPin, LOW);
    analogWrite(pwmMotorOnePin,topSpeed); 
  
    //set motor two direction and pwm pulse rate 
    digitalWrite(motorTwoDirectionPin, LOW); 
    analogWrite(pwmMotorTwoPin, (topSpeed/2));
  
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
    if(digitalRead(joystickRightPin) == 1){
      if(digitalRead(joystickBackwardPin) == 1){
        brake();
      }  
    }
    if(bluetoothSerial.available()){
      bluetoothBrake();  
    }//end original code 
  }
}



