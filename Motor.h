#ifndef MOTOR_H
#define MOTOR_H

#include <AccelStepper.h>

class Articulacion{
    public:
      AccelStepper motor;
      int step_pin;
      int dir_pin;
      int sensor_pin;
      float steps_per_unit;
      float min_pos;
      float max_pos;
      float max_speed;
      int offSet;
      int currentPosition;

Articulacion(
    int _dir_pin,
    int _step_pin,
    int _sensor_pin,
    float _steps_per_unit
    // const float _min_pos,
    // const float _max_pos,
    // const float _max_speed
) : motor(AccelStepper(AccelStepper::DRIVER, _step_pin, _dir_pin)) {
    step_pin = _step_pin;
    dir_pin = _dir_pin;
    steps_per_unit = _steps_per_unit;
    pinMode(_sensor_pin, INPUT);
    // min_pos = _min_pos;
    // max_pos = _max_pos;
    max_speed = 1000;
    sensor_pin = _sensor_pin;
    motor.setMaxSpeed(30000.0);
    motor.setAcceleration(100.0);
};


    void setOffset(){
      bool offSetFlag = false;
      while(!offSetFlag){
        motor.moveTo((int)(360*steps_per_unit));
        motor.setSpeed(50*steps_per_unit);
        motor.runSpeedToPosition();
        offSetFlag = !digitalRead(sensor_pin);
      }
      motor.stop();
      motor.setSpeed(0);
      offSet = motor.currentPosition();
    }
  
    void setSpeed(float motor_speed){
      motor.setMaxSpeed(motor_speed * steps_per_unit);
    };
    void setAccel(float motor_accel){
      motor.setAcceleration(motor_accel*steps_per_unit);
    };

    void moveTo(float move_target,float speed_target){
      int target = (int)(move_target*steps_per_unit);
      motor.moveTo(target + offSet);
      motor.setMaxSpeed(speed_target*steps_per_unit);
      motor.setSpeed(speed_target*steps_per_unit);
    };

    void calibrate(float positionTarget, float speedTarget){
      moveTo(positionTarget, speedTarget);
      unsigned long startTime = millis();
      unsigned long lastSerialPrintTime = 0;
    
    while (true) {
        motor.runSpeedToPosition();
        if (motor.currentPosition() == floor(abs(positionTarget)*steps_per_unit)) {
            Serial.println(millis()-startTime);
            break;
        }
        if(millis()-startTime > 10000){
          break;
        }
    }
    Serial.println("Calibration complete");
    }






};
#endif