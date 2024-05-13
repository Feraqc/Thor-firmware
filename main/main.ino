#include "Motor.h"

// Articulacion 1
#define dirPinA1 5
#define stepPinA1 2
#define sensorPin1 43
#define stepsPerUnit1 2.7777

// // Articulacion 2
#define dirPin2A 6
#define stepPin2A 3
#define dirPin2B 7
#define stepPin2B 4
#define sensorPin2 0
#define stepsPerUnit2 14.8

//Articulacion 3

// #define dirPin3 6
// #define stepPin3 3
// #define dirPin3 7
// #define stepPin3 4
// #define sensorPin3 0
// #define stepsPerUnit3 14.8



float position;
float speed;
unsigned long sum1=0;
unsigned long sum2=0;
Articulacion articulacion1(dirPinA1,stepPinA1,sensorPin1,stepsPerUnit1);
Articulacion articulacion2(dirPin2A,stepPin2A,dirPin2B,stepPin2B,sensorPin2,stepsPerUnit2);
//Articulacion articulacion3(dirPinA1,stepPinA1,sensorPin1,stepsPerUnit1);



void setup() {
  Serial.begin(9600);
  pinMode(8,OUTPUT);
  digitalWrite(8,0);
  //articulacion1.setOffset();
  //articulacion2.setOffset();
}
void loop() {

  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
    Serial.print(position);
    Serial.print(",");
    Serial.println(speed);
    articulacion2.moveTo(position, speed);
    //articulacion2.moveTo(position, speed);
    // sum1 = 0;
    // for(int i=0;i<1;i++){
    //   sum1 = articulacion1.calibrate(position,speed);
    //   Serial.println("end");
    // }
    // Serial.print(sum1);
    // sum1 = 0;
  }
    articulacion2.motor1.runSpeedToPosition();
    articulacion2.motor2.runSpeedToPosition();
  

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