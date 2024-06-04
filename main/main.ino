#include "Motor.h"

// Articulacion 1
#define dirPinA1 36
#define stepPinA1 28
#define sensorPin1 42
#define stepsPerUnit1 44.4444//2.7777

// // Articulacion 2
#define dirPin2A 34
#define stepPin2A 26
#define dirPin2B 32
#define stepPin2B 24
#define sensorPin2 44
#define stepsPerUnit2 236.8

//Articulacion 3
 #define dirPin3 30
 #define stepPin3 22
 #define sensorPin3 46
 #define stepsPerUnit3 236.8

//Articulacion 4
#define dirPin4 31
#define stepPin4 23
#define sensorPin4 48
#define stepsPerUnit4 17.7777

//Articulacion 5
#define dirPin5 33
#define stepPin5 25
#define sensorPin5 49
#define stepsPerUnit5 17.7777

//Articulacion 6
#define dirPin6 35
#define stepPin6 27
#define sensorPin6 47
#define stepsPerUnit6 17.7777

//step8 29
//dir8 37
//sensor7 45
//sensor8 43



float position;
float speed;
int motorTarget;

Articulacion articulacion1(dirPinA1,stepPinA1,sensorPin1,stepsPerUnit1);
Articulacion articulacion2(dirPin2A,stepPin2A,dirPin2B,stepPin2B,sensorPin2,stepsPerUnit2);
Articulacion articulacion3(dirPin3,stepPin3,sensorPin3,stepsPerUnit3);
Articulacion articulacion4(dirPin4,stepPin4,sensorPin4,stepsPerUnit4);
Articulacion articulacion5(dirPin5,stepPin5,sensorPin5,stepsPerUnit5);
Articulacion articulacion6(dirPin6,stepPin6,sensorPin6,stepsPerUnit6);

void setup() {
  pinMode(8,OUTPUT);
  digitalWrite(8,0);
  Serial.begin(9600);
  Serial.println("enter");
  delay(1000);
  //articulacion1.setOffSet360();
  //articulacion2.setOffSet180();
}
void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
    Serial.print(motorTarget);
    Serial.print(",");
    Serial.print(position);
    Serial.print(",");
    Serial.println(speed);
        switch(motorTarget){
        case 1:
          articulacion1.moveTo(position, speed);
          break;
        case 2:
          articulacion2.moveTo(position, speed);
          break;
        case 3:
          articulacion3.moveTo(position, speed);
          break;
        case 4:
          articulacion4.moveTo(position, speed);
          break;
        case 5:
          articulacion5.moveTo(position, speed);
          break;
        case 6:
          articulacion6.moveTo(position, speed);
          break;
        }
  }
  //Serial.println(digitalRead(articulacion2.sensorPin));
  articulacion1.motor1.runSpeedToPosition();
  articulacion2.motor1.runSpeedToPosition();
  articulacion2.motor2.runSpeedToPosition();
  articulacion3.motor1.runSpeedToPosition();
  articulacion4.motor1.runSpeedToPosition();
  articulacion5.motor1.runSpeedToPosition();
  articulacion6.motor1.runSpeedToPosition();
}

void parseCommand(String cmd) {
  int firstCommaIndex = cmd.indexOf(",");
  if (firstCommaIndex != -1) { 
    int secondCommaIndex = cmd.indexOf(",", firstCommaIndex + 1);
    if (secondCommaIndex != -1) {
      String variable1String = cmd.substring(0, firstCommaIndex);
      String variable2String = cmd.substring(firstCommaIndex + 1, secondCommaIndex);
      String variable3String = cmd.substring(secondCommaIndex + 1);
    motorTarget = variable1String.toInt(); 
    position = variable2String.toInt(); 
    speed = variable3String.toInt();
  }
  }
}
