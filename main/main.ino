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
//Esta mal
// #define dirPin3 30
// #define stepPin3 22
// #define dirPin3 7
// #define stepPin3 4
// #define sensorPin3 0
// #define stepsPerUnit3 



float position;
float speed;
int motorTarget;

unsigned long sum1=0;
unsigned long sum2=0;
Articulacion articulacion1(dirPinA1,stepPinA1,sensorPin1,stepsPerUnit1);
Articulacion articulacion2(dirPin2A,stepPin2A,dirPin2B,stepPin2B,sensorPin2,stepsPerUnit2);
Articulacion articulacion3(dirPin3,stepPin3,sensorPin3,stepsPerUnit3);



void setup() {
  pinMode(8,OUTPUT);
  digitalWrite(8,0);
  Serial.begin(9600);
  Serial.println("enter");
  //articulacion1.setOffset();
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
        switch(motorTarget) {
        case 1:
            articulacion1.moveTo(position, speed);
            break;
        case 2:

            articulacion2.moveTo(position, speed);
            break;
        case 3:
            articulacion3.moveTo(position, speed);
            break;
        }
  }
  Serial.println(digitalRead(articulacion1.sensorPin));
  articulacion1.motor1.runSpeedToPosition();
  //articulacion2.motor1.runSpeedToPosition();
  //articulacion2.motor2.runSpeedToPosition();
  //articulacion3.motor1.runSpeedToPosition();


  

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
