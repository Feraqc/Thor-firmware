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
#define stepsPerUnit2 270

//Articulacion 3
 #define dirPin3 30
 #define stepPin3 22
 #define sensorPin3 49
 #define stepsPerUnit3 275

//Articulacion 4
#define dirPin4 35//31
#define stepPin4 27//23
#define sensorPin4 46
#define stepsPerUnit4 17.7777

//Articulacion 5
#define dirPin5 33
#define stepPin5 25
#define sensorPin5 43
#define stepsPerUnit5 8.2//17.7777

//Articulacion 6 NO SE USA
#define dirPin6 31//35
#define stepPin6 23//27
#define sensorPin6 47
#define stepsPerUnit6 44.4444//17.7777

//Articulacion 7 NO SE USA
#define dirPin7 37
#define stepPin7 29
#define sensorPin7 47
#define stepsPerUnit7 44.4444

float q1,q2,q3,q4,q5,q6;
float speed;
//int motorTarget;

Articulacion articulacion1(dirPinA1,stepPinA1,sensorPin1,stepsPerUnit1);
Articulacion articulacion2(dirPin2A,stepPin2A,dirPin2B,stepPin2B,sensorPin2,stepsPerUnit2);
Articulacion articulacion3(dirPin3,stepPin3,sensorPin3,stepsPerUnit3);
Articulacion articulacion4(dirPin4,stepPin4,sensorPin4,stepsPerUnit4);
Articulacion articulacion5(dirPin5,stepPin5,sensorPin5,stepsPerUnit5);
Articulacion articulacion6(dirPin6,stepPin6,sensorPin6,stepsPerUnit6);
Articulacion articulacion7(dirPin7,stepPin7,sensorPin7,stepsPerUnit7);

void setup() {
  pinMode(8,OUTPUT);
  digitalWrite(8,0);
  Serial.begin(9600);
  Serial.println("begin");
  //Debemos anhadir la busqueda al cero 
  delay(1000);
  articulacion2.Art2Ofsset();
  articulacion1.Art360Ofsset();
  articulacion3.Art3Ofsset();
  articulacion4.Art360Ofsset();
}

void loop() {
  if (Serial.available() > 0) {
    String command = Serial.readStringUntil('\n');
    parseCommand(command);
    Serial.print(q1);
    Serial.print(",");
    Serial.print(q2);
    Serial.print(",");
    Serial.print(q3);
    Serial.print(",");
    Serial.print(q4);
    Serial.print(",");
    Serial.print(q5);
    Serial.print(",");
    Serial.print(q6);
    Serial.print(",");
    Serial.println(speed);

    articulacion1.moveTo(q1, speed);
    articulacion2.moveTo(q2, speed);
    articulacion3.moveTo(q3, speed);
    articulacion4.moveTo(q4, speed);
  }
  articulacion1.motor1.runSpeedToPosition();
  articulacion2.motor1.runSpeedToPosition();
  articulacion2.motor2.runSpeedToPosition();
  articulacion3.motor1.runSpeedToPosition();
  articulacion4.motor1.runSpeedToPosition();
  /*articulacion4.motor1.runSpeedToPosition();
  articulacion5.motor1.runSpeedToPosition();*/
}

/*void parseCommand(String cmd) {
    int firstCommaIndex = cmd.indexOf(",");
    if (firstCommaIndex != -1) {
        int secondCommaIndex = cmd.indexOf(",", firstCommaIndex + 1);
        if (secondCommaIndex != -1) {
            int thirdCommaIndex = cmd.indexOf(",", secondCommaIndex + 1);
            if (thirdCommaIndex != -1) {
                String variable1String = cmd.substring(0, firstCommaIndex);
                String variable2String = cmd.substring(firstCommaIndex + 1, secondCommaIndex);
                String variable3String = cmd.substring(secondCommaIndex + 1, thirdCommaIndex);
                String variable4String = cmd.substring(thirdCommaIndex + 1);

                q1 = variable1String.toFloat();
                q2 = variable2String.toFloat();
                q3 = variable3String.toFloat();
                speed = variable4String.toFloat();
            }
        }
    }
}*/
void parseCommand(String cmd) {
  int commaIndex[6];
  int i = 0;
  int lastCommaIndex = -1;

  // Encontrar todas las comas
  while (i < 6) {
    int nextCommaIndex = cmd.indexOf(",", lastCommaIndex + 1);
    if (nextCommaIndex == -1) break;
    commaIndex[i] = nextCommaIndex;
    lastCommaIndex = nextCommaIndex;
    i++;
  }

  if (i == 6) { // Asegurarse de que se encontraron exactamente 6 comas
    String q1String = cmd.substring(0, commaIndex[0]);
    String q2String = cmd.substring(commaIndex[0] + 1, commaIndex[1]);
    String q3String = cmd.substring(commaIndex[1] + 1, commaIndex[2]);
    String q4String = cmd.substring(commaIndex[2] + 1, commaIndex[3]);
    String q5String = cmd.substring(commaIndex[3] + 1, commaIndex[4]);
    String q6String = cmd.substring(commaIndex[4] + 1, commaIndex[5]);
    String speedString = cmd.substring(commaIndex[5] + 1);

    q1 = q1String.toInt(); 
    q2 = q2String.toInt(); 
    q3 = q3String.toInt(); 
    q4 = q4String.toInt();
    q5 = q5String.toInt();
    q6 = q6String.toInt();
    speed = speedString.toInt();

    if (speed > 20) {
      Serial.println("[Velocidad Fuera de Rango]");
      return;
    }

    Serial.print(q1);
    Serial.print(",");
    Serial.print(q2);
    Serial.print(",");
    Serial.print(q3);
    Serial.print(",");
    Serial.print(q4);
    Serial.print(",");
    Serial.print(q5);
    Serial.print(",");
    Serial.print(q6);
    Serial.print(",");
    Serial.println(speed);
    
    articulacion1.moveTo(q1, speed);
    articulacion2.moveTo(q2, speed);
    articulacion3.moveTo(q3, speed);
    articulacion4.moveTo(q4, speed);
    articulacion5.moveTo(q5, speed);
    articulacion6.moveTo(q6, speed);
  } else {
    Serial.println("[Comando Invalido]");
  }
}


