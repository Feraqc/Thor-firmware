#include "Motor.h"

#define dirPin1 36
#define stepPin1 28
#define sensorPin1 43
#define stepsPerUnit1 2.7777

float position;
float speed;
unsigned long sum1=0;
unsigned long sum2=0;
Articulacion articulacion1(dirPin1,stepPin1,sensorPin1,stepsPerUnit1);

void setup() {
  Serial.begin(9600);
  articulacion1.setOffset();

}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
    Serial.print(position);
    Serial.print(",");
    Serial.println(speed);
    //articulacion1.moveTo(position,speed);
    for(int i=0;i<5;i++){
      sum1 = sum1 + articulacion1.calibrate(360,speed);
      sum2 = sum2 + articulacion1.calibrate(0,speed);
    }
    Serial.println(sum1/5);
    Serial.println(sum2/5);
  }
  //articulacion1.motor.runSpeedToPosition();
  
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