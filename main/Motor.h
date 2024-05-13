#ifndef MOTOR_H
#define MOTOR_H

#include <AccelStepper.h>

class Articulacion{
    public:
      AccelStepper motor1;
      AccelStepper motor2;
      int stepPin1;
      int dirPin1;
      int stepPin2;
      int dirPin2;
      int sensorPin;
      float steps_per_unit;
      float minPos;
      float maxPos;
      float maxSpeed;
      int offSet;
      int currentPosition;

// CONSTRUCTOR UN MOTOR
Articulacion(
    int _dirPin1,
    int _stepPin1,
    int _sensorPin,
    float _steps_per_unit): 
    motor1(AccelStepper(AccelStepper::DRIVER, _stepPin1, _dirPin1)){
    stepPin1 = _stepPin1;
    dirPin1 = _dirPin1;
    stepPin2 = -1;
    dirPin2 = -1;
    steps_per_unit = _steps_per_unit;
    pinMode(_sensorPin, INPUT);
    maxSpeed = 1000;
    sensorPin = _sensorPin;
    motor1.setMaxSpeed(1000.0);
    motor1.setAcceleration(250.0);
}

// CONSTRUCTOR DOS MOTORES
Articulacion(
    int _dirPin1,
    int _stepPin1,
    int _dirPin2,
    int _stepPin2,
    int _sensorPin,
    float _steps_per_unit
) : motor1(AccelStepper(AccelStepper::DRIVER, _stepPin1, _dirPin1)),
    motor2(AccelStepper(AccelStepper::DRIVER, _stepPin2, _dirPin2)){
    stepPin1 = _stepPin1;
    dirPin1 = _dirPin1;
    stepPin2 = _stepPin2;
    dirPin2 = _dirPin2;
    steps_per_unit = _steps_per_unit;
    pinMode(_sensorPin, INPUT);
    maxSpeed = 1000;
    sensorPin = _sensorPin;
    motor1.setMaxSpeed(30000.0);
    motor1.setAcceleration(50.0);
    motor2.setMaxSpeed(30000.0);
    motor2.setAcceleration(10.0);
}


    void setOffset(){
      Serial.println("Setting offSet");
      bool offSetFlag = false;
      moveTo(360,10);
      while(!offSetFlag){
        motor1.runSpeedToPosition();
        if(stepPin2 != -1 && dirPin2 != -1){
          motor2.runSpeedToPosition();
        }
        offSetFlag = !digitalRead(sensorPin);
      }
      motor1.stop();
      motor1.setSpeed(0);
      if(stepPin2 != -1 && dirPin2 != -1){
        motor2.stop();
        motor2.setSpeed(0);
      }
      offSet = motor1.currentPosition();
      maxPos = motor1.currentPosition();

      if(stepPin2 != -1 && dirPin2 != -1){
        bool offSetFlag2 = false;
        moveTo(-2*360,10);
        while(!offSetFlag2){
          static unsigned long secondOffset = millis();
          motor1.runSpeedToPosition();
          motor2.runSpeedToPosition();
          if(millis() - secondOffset >= 2000){
            offSetFlag2 = !digitalRead(sensorPin);
          }
        }
        motor1.stop();
        motor1.setSpeed(0);
        if(stepPin2 != -1 && dirPin2 != -1){
          motor2.stop();
          motor2.setSpeed(0);
        }
          minPos = motor1.currentPosition();
      }

      Serial.print(maxPos);
      Serial.print(",");
      Serial.println(minPos);

      Serial.println("End offSet");
    }
  
    void setSpeed(float motor_speed){
      motor1.setMaxSpeed(motor_speed * steps_per_unit);
      if(stepPin2 != -1 && dirPin2 != -1){
        motor2.setMaxSpeed(motor_speed * steps_per_unit);
      }
    };

    void setAccel(float motor_accel){
      motor1.setAcceleration(motor_accel*steps_per_unit);
      if(stepPin2 != -1 && dirPin2 != -1){
        motor2.setAcceleration(motor_accel*steps_per_unit);
      }
    };

    // falta offset
    void moveTo(float move_target,float speed_target){
      int target = (int)(move_target*steps_per_unit);
      motor1.moveTo(target);
      //motor1.setMaxSpeed(speed_target*steps_per_unit);
      //motor1.setAcceleration(10*steps_per_unit);
      motor1.setSpeed(speed_target*steps_per_unit);
      if(stepPin2 != -1 && dirPin2 != -1){
        motor2.moveTo(target);
        motor2.setSpeed(speed_target*steps_per_unit);
      }
    };

    unsigned long calibrate(float positionTarget, float speedTarget){
      moveTo(positionTarget,speedTarget);
      unsigned long serialTime = millis();
      unsigned long startTime = millis();
      while (true) {
       // motor1.run();
     motor1.runSpeedToPosition();
        // if(millis()-serialTime >= 500){
        //   serialTime = millis();
        //   Serial.println(motor1.currentPosition());
        // }
        if (abs(motor1.currentPosition()) == floor(abs(positionTarget)*steps_per_unit)+offSet) {
          return (millis()-startTime);
            break;
        }
        if(millis()-startTime > 25000){
          break;
        }
      }
    }






};
#endif