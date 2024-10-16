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
      int minPos;
      int maxPos;
      int midPos;
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
          motor1.setMaxSpeed(30000.0);
          motor1.setAcceleration(500);
      }
      // CONSTRUCTOR DOS MOTORES
      Articulacion(
          int _dirPin1,
          int _stepPin1,
          int _dirPin2,
          int _stepPin2,
          int _sensorPin,
          float _steps_per_unit) 
          :motor1(AccelStepper(AccelStepper::DRIVER, _stepPin1, _dirPin1)),
          motor2(AccelStepper(AccelStepper::DRIVER, _stepPin2, _dirPin2)){
          stepPin1 = _stepPin1;
          dirPin1 = _dirPin1;
          stepPin2 = _stepPin2;
          dirPin2 = _dirPin2;
          steps_per_unit = _steps_per_unit;
          pinMode(_sensorPin, INPUT);
          sensorPin = _sensorPin;
          motor1.setMaxSpeed(10000.0);
          motor1.setAcceleration(10.0);
          motor2.setMaxSpeed(10000.0);
          motor2.setAcceleration(10.0);
      }


      void setOffSet360(){
        Serial.println("Setting offSet");
        //FIRST TURN
        bool offSetFlag = false;
        moveTo(360*steps_per_unit,20);
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
        maxPos = offSet;
        //SECOND TURN
        offSetFlag = false;
        moveTo(-2*360*steps_per_unit,20);
        while(!offSetFlag){
          static unsigned long startTime = millis();
          motor1.runSpeedToPosition();
          if(stepPin2 != -1 && dirPin2 != -1){
            motor2.runSpeedToPosition();
          }
          if(millis()-startTime > 1000){
            offSetFlag = !digitalRead(sensorPin);
          }
        }
        motor1.stop();
        motor1.setSpeed(0);
        if(stepPin2 != -1 && dirPin2 != -1){
          motor2.stop();
          motor2.setSpeed(0);
        }
        minPos = motor1.currentPosition()+offSet;
        Serial.println("End offSet");
        Serial.print(minPos);
        Serial.print(",");
        Serial.println(maxPos);
      }

      void setOffSet180(){
        Serial.println("Setting offSet");
        moveTo(360*steps_per_unit,20);
         bool offSetFlag1 = false;
        //FIRST LIMIT
        while(true){
          delay(100);
          motor1.runSpeedToPosition();
          delay(100);
          motor2.runSpeedToPosition();
          if(!digitalRead(sensorPin)){
            break;
          }
        }
        Serial.println("salio");
        motor1.stop();
        motor1.setSpeed(0);
        if(stepPin2 != -1 && dirPin2 != -1){
          motor2.stop();
          motor2.setSpeed(0);
        }
        offSet = motor1.currentPosition();
        maxPos = offSet;

        // //CENTRAL LIMIT
        // bool offSetFlag2 = false;
        // moveTo(-180*steps_per_unit,20);
        // while(!offSetFlag2){
        //   static unsigned long startTime2 = millis();
        //   motor1.runSpeedToPosition();
        //   if(stepPin2 != -1 && dirPin2 != -1){
        //     motor2.runSpeedToPosition();
        //   }
        //   if(millis()-startTime2 > 2000){
        //     offSetFlag2 = !digitalRead(sensorPin);
        //   }
        // }
        // motor1.stop();
        // motor1.setSpeed(0);
        // if(stepPin2 != -1 && dirPin2 != -1){
        //   motor2.stop();
        //   motor2.setSpeed(0);
        // }
        // midPos = motor1.currentPosition();

        // //LAST LIMIT
        // bool offSetFlag3 = false;
        // moveTo(-360*steps_per_unit,20);
        // while(!offSetFlag3){
        // static unsigned long startTime3 = millis();
        //   motor1.runSpeedToPosition();
        //   if(stepPin2 != -1 && dirPin2 != -1){
        //     motor2.runSpeedToPosition();
        //   }
        //   if(millis()-startTime3 > 2000){
        //     offSetFlag3 = !digitalRead(sensorPin);
        //   }
        // }
        // motor1.stop();
        // motor1.setSpeed(0);
        // if(stepPin2 != -1 && dirPin2 != -1){
        //   motor2.stop();
        //   motor2.setSpeed(0);
        // }
        // maxPos = motor1.currentPosition();

        Serial.println("End offSet");
        Serial.print(minPos);
        Serial.print("-");
        Serial.print(midPos);
        Serial.print("-");
        Serial.println(maxPos);

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

      void moveTo(float move_target,float speed_target){
        int target = (int)(move_target*steps_per_unit);
        motor1.moveTo(target+offSet);
        motor1.setSpeed(speed_target*steps_per_unit);
        if(stepPin2 != -1 && dirPin2 != -1){
          motor2.moveTo(target+offSet);
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

      void run(){
            static unsigned long startTime = millis();
            motor1.setAcceleration(25);     
            if(millis()-startTime <= 500){
                motor1.run();
            }
            else{
              motor1.runSpeedToPosition();     
            }
      }







};
#endif
