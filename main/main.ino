#include "Motor.h"

// Articulacion 1
#define dirPinA1 5
#define stepPinA1 2
#define sensorPin1 9
#define stepsPerUnit1 2.7777

// // Articulacion 2
// #define dirPin2A 34
// #define stepPin2A 26
// #define dirPin2B 32
// #define stepPin2B 24
// #define sensorPin2 44
// #define stepsPerUnit2 236.8



float position;
float speed;
unsigned long sum1=0;
unsigned long sum2=0;
Articulacion articulacion1(dirPinA1,stepPinA1,sensorPin1,stepsPerUnit1);
//Articulacion articulacion2(dirPin2A,stepPin2A,dirPin2B,stepPin2B,sensorPin2,stepsPerUnit2);


void setup() {
  Serial.begin(9600);
  //articulacion1.setOffset();
  //articulacion2.setOffset();
  pinMode(8,OUTPUT);
  digitalWrite(8,LOW);
  pinMode(dirPinA1,OUTPUT);
  pinMode(stepPinA1,OUTPUT);

  articulacion1.offSet = articulacion1.motor1.currentPosition();
  Serial.println(articulacion1.offSet);
}

void loop() {

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
    Serial.print(position);
    Serial.print(",");
    Serial.println(speed);
    //articulacion1.moveTo(position, speed);
    //articulacion2.moveTo(position, speed);
    sum1 = 0;
    for(int i=0;i<1;i++){
      sum1 = articulacion1.calibrate(position,speed);
      Serial.println("end");
    }
    Serial.print(sum1);
    sum1 = 0;
  }
    //articulacion1.motor1.runSpeedToPosition();
    //articulacion2.motor1.runSpeedToPosition();
    //articulacion1.motor1.runSpeedToPosition();
  

}

void parseCommand(String cmd) {
  int commaIndex = cmd.indexOf(",");
  if (commaIndex != -1) { 
    String variable1String = cmd.substring(0, commaIndex); 
    String variable2String = cmd.substring(commaIndex + 1); 
    position = variable1String.toInt(); 
    speed = variable2String.toInt();
  }
}