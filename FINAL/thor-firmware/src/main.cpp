#include <Arduino.h>
#include <Motor.h>
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
 #define sensorPin3 46
 #define stepsPerUnit3 275

//Articulacion 4
#define dirPin4 31
#define stepPin4 23
#define sensorPin4 48
#define stepsPerUnit4 17.7777

//Articulacion 5
#define dirPin5 33
#define stepPin5 25
#define sensorPin5 49
#define stepsPerUnit5 35

//Articulacion 6
#define dirPin6 35
#define stepPin6 27
#define sensorPin6 47
#define stepsPerUnit6 35

Articulacion articulacion1(dirPinA1,stepPinA1,sensorPin1,stepsPerUnit1);
Articulacion articulacion2(dirPin2A,stepPin2A,dirPin2B,stepPin2B,sensorPin2,stepsPerUnit2);
Articulacion articulacion3(dirPin3,stepPin3,sensorPin3,stepsPerUnit3);
Articulacion articulacion4(dirPin4,stepPin4,sensorPin4,stepsPerUnit4);
Articulacion articulacion5(dirPin5,stepPin5,sensorPin5,stepsPerUnit5);
Articulacion articulacion6(dirPin6,stepPin6,sensorPin6,stepsPerUnit6);


void handleWaitOnCommand();
void handleGo2Home();
void handleSetHome();
void handleMoveSpeed();
void handleMoveAcceleration();
void handleSetTrajectory();
void handleGoTrajectory();
void handleGoPose();
void handleEmergencyStop();
void handleInput(String cmd);
void handleMoveWrist();



enum State {
  WAIT_ON_COMAND,
  EMERGENCY_STOP,
  GO_TO_HOME,
  SET_HOME,
  MOVE_CONSTANT_SPEED,
  MOVE_ACCELERATION,
  SET_TRAJECTORY,
  GO_TRAJECTORY,
  GO_POSE,
  MOVE_WRIST
};

State currentState;

  const int maxCommands = 91;
  // Arrays to store parsed commands
  String parsedCommands[maxCommands];
  String function;
  int functionParameters[maxCommands-1];
  int commandCount = 0;

  // VARIABLES ARTICULARES
  int q[6];

void setup() {
  pinMode(8,OUTPUT);
  digitalWrite(8,0);
  Serial.begin(115200);
  currentState = WAIT_ON_COMAND;

}

void loop() {
  switch(currentState){
    case WAIT_ON_COMAND:
      handleWaitOnCommand();
    break;

    case EMERGENCY_STOP:
      //handleEmergencyStop();
    break;

    case GO_TO_HOME:
      handleGo2Home();
    break;

    case SET_HOME:
      //handleSetHome();
    break;

    case MOVE_CONSTANT_SPEED:
      handleMoveSpeed();
    break;

    case MOVE_ACCELERATION:
      handleMoveAcceleration();
    break;

    case SET_TRAJECTORY:
      //handleSetTrajectory(); 
    break;

    case GO_TRAJECTORY:
      //handleGoTrajectory();
    break;

    case GO_POSE:
      //handleGoPose();
    break;

    case MOVE_WRIST:
      handleMoveWrist();
    break;
  }
}

void resetArrays(){
  // Clear the parsedCommands array
  for (int i = 0; i < maxCommands; i++) {
    parsedCommands[i] = "";
  }

  // Clear the functionParameters array
  for (int i = 0; i < maxCommands - 1; i++) {
    functionParameters[i] = 0;
  }
}

void handleWaitOnCommand(){
  if (Serial.available() > 0) {
    String cmd = Serial.readStringUntil('\n'); // Read the input string until newline
    handleInput(cmd);

    if (commandCount > 0) {
      function = parsedCommands[0];
      function.trim();
      Serial.print("FUNCTION: ");
      Serial.println(function);

      Serial.print("PARAMETERS: ");
      for (int i = 0; i < commandCount - 1; i++) {
        parsedCommands[i + 1].trim();
        functionParameters[i] = parsedCommands[i + 1].toInt();
        Serial.print(functionParameters[i]);
        Serial.print("\t");
      }
      Serial.println();

      // Serial.print("Command count: ");
      // Serial.println(commandCount);

      if (function.equals("G00")) {
        Serial.println("GOING TO HOME");
        currentState = GO_TO_HOME;
      } else if (function.equals("S00")) {
        Serial.println("SETTING HOME");
        currentState = SET_HOME;
      } else if (function.equals("G2")) {
        Serial.println("MOVING OBJECTIVE JOINT AT CONSTANT SPEED");
        currentState = MOVE_CONSTANT_SPEED;
      } else if (function.equals("G13")) {
        Serial.println("MOVING OBJECTIVE JOINT WITH ACCELERATION");
        currentState = MOVE_ACCELERATION; // Set the appropriate state if defined
      } else if (function.equals("G1")) {
        Serial.println("MOVING WRIST OBJECTIVE POINT");
        currentState = MOVE_WRIST; // Set the appropriate state if defined
      } else if (function.equals("STR")) {
        Serial.println("SETTING TRAJECTORIES POINTS");
        // currentState = SET_TRAJECTORY; // Set the appropriate state if defined
      } else if (function.equals("GTR")) {
        Serial.println("FOLLOWING TRAJECTORY");
        // currentState = FOLLOW_TRAJECTORY; // Set the appropriate state if defined
      } else if (function.equals("P1")) {
        Serial.println("MOVING TOOL TO OBJECTIVE POINT");
        // currentState = MOVE_TOOL; // Set the appropriate state if defined
      } else {
        Serial.println("Unknown command.");
      }

    }
  }
}

void handleInput(String cmd) {
  resetArrays();
  commandCount = 0; // Reset command count
  unsigned int startIndex = 0;
  int spaceIndex = cmd.indexOf(" ");

  while (spaceIndex != -1 && commandCount < maxCommands) {
    String commandString = cmd.substring(startIndex, spaceIndex);
    parsedCommands[commandCount] = commandString;
    commandCount++;
    startIndex = spaceIndex + 1;
    spaceIndex = cmd.indexOf(" ", startIndex);
  }

  // Handle the last command (or the only command if no spaces were found)
  if (startIndex < cmd.length() && commandCount < maxCommands) {
    parsedCommands[commandCount] = cmd.substring(startIndex);
    commandCount++;
  }
}

void handleGo2Home(){

  articulacion1.moveTo(0,10);
  articulacion2.moveTo(0,10);
  articulacion3.moveTo(0,10);
  articulacion4.moveTo(0,10);

  articulacion1.motor1.runSpeedToPosition();
  articulacion2.motor1.runSpeedToPosition();
  articulacion2.motor2.runSpeedToPosition();
  articulacion3.motor1.runSpeedToPosition();
  articulacion4.motor1.runSpeedToPosition();

}

void handleMoveSpeed(){
  //Serial.println("MOVING OBJECTIVE JOINT AT CONSTANT SPEED");
  int joint = functionParameters[0];
  int alpha = functionParameters[1];
  int omega = functionParameters[2];
  if(joint == 1){
    articulacion1.moveTo(alpha,omega);
    while (articulacion1.motor1.distanceToGo() != 0){
      articulacion1.motor1.runSpeedToPosition();
    }
    currentState = WAIT_ON_COMAND;
    Serial.println("WAITING COMMAND");
  }

  else if(joint == 2){
    articulacion2.moveTo(alpha,omega);
    while (articulacion2.motor1.distanceToGo() != 0 && articulacion2.motor2.distanceToGo() != 0){
      articulacion2.motor1.runSpeedToPosition();
      articulacion2.motor2.runSpeedToPosition();
    }
    currentState = WAIT_ON_COMAND;
    Serial.println("WAITING COMMAND");
  }
  else if(joint == 3){
    articulacion3.moveTo(alpha,omega);
    while (articulacion3.motor1.distanceToGo() != 0){
      articulacion3.motor1.runSpeedToPosition();
    }
    currentState = WAIT_ON_COMAND;
    Serial.println("WAITING COMMAND");
  }
  else if(joint == 4){
    articulacion4.moveTo(alpha,omega);
    while (articulacion4.motor1.distanceToGo() != 0){
      articulacion4.motor1.runSpeedToPosition();
    }
    currentState = WAIT_ON_COMAND;
    Serial.println("WAITING COMMAND");
  }
  else if(joint == 5){
    articulacion5.moveTo(alpha,omega);
    articulacion6.moveTo(-alpha,omega);
    while (articulacion5.motor1.distanceToGo() != 0 && articulacion6.motor1.distanceToGo() != 0){
      articulacion5.motor1.runSpeedToPosition();
      articulacion6.motor1.runSpeedToPosition();
    }
    currentState = WAIT_ON_COMAND;
    Serial.println("WAITING COMMAND"); 
  }
  else if(joint == 6){    
    articulacion5.moveTo(alpha,omega);
    articulacion6.moveTo(alpha,omega);
    while (articulacion5.motor1.distanceToGo() != 0 && articulacion6.motor1.distanceToGo() != 0){
      articulacion5.motor1.runSpeedToPosition();
      articulacion6.motor1.runSpeedToPosition();
    }
    currentState = WAIT_ON_COMAND;
    Serial.println("WAITING COMMAND"); 

    }
  else{
    Serial.println("JOINT NOT EXIST");
    currentState = WAIT_ON_COMAND;
  }
}

void handleMoveAcceleration(){

  int joint = functionParameters[0];
  int alpha = functionParameters[1];
  int omega = functionParameters[2]; 
  int beta1 = functionParameters[3];
  int beta2 = functionParameters[4];
  if(joint == 1){
    
    alpha = alpha*articulacion1.steps_per_unit;
    omega = omega*articulacion1.steps_per_unit;
    beta1 = beta1*articulacion1.steps_per_unit;
    beta2 = beta2*articulacion1.steps_per_unit;

    //float acc = 2*(abs(articulacion1.motor1.currentPosition()-beta1))/25;
   // articulacion1.motor1.setAcceleration(acc);
    if(alpha < articulacion1.motor1.currentPosition()){
      omega = -omega;
      beta1 = -beta1;
      beta2 = -beta2;
    }
    articulacion1.motor1.setMaxSpeed(omega);
    articulacion1.motor1.moveTo(alpha);
    while (articulacion1.motor1.distanceToGo() < beta1){
      articulacion1.motor1.run();
    }
    Serial.println("BETA1 REACHED");
    //articulacion1.moveTo(beta2,omega);
    articulacion1.motor1.moveTo(alpha);
    articulacion1.motor1.setSpeed(omega);
    while (articulacion1.motor1.distanceToGo() < beta2){
      articulacion1.motor1.runSpeedToPosition();
    }
    //float acc2 = 2*(abs(abs(alpha - articulacion1.motor1.currentPosition()) - 5*omega))/25;
    //articulacion1.motor1.setAcceleration(acc2);
    Serial.println("BETA2 REACHED");
    //articulacion1.motor1.moveTo(alpha*articulacion1.steps_per_unit);
    while (articulacion1.motor1.distanceToGo() != 0){
      articulacion1.motor1.run();
    }
    Serial.println("ALPHA REACHED");
    currentState = WAIT_ON_COMAND;
    Serial.println("WAITING COMMAND");
  }
  else{
    Serial.println("JOINT NOT EXIST");
    currentState = WAIT_ON_COMAND;
  }  

}

void handleMoveWrist(){
  int q1 = functionParameters[0];
  int q2 = functionParameters[2];
  int q3 = functionParameters[3];
  int movType = functionParameters[4];

  articulacion1.moveTo(q1,10);
  articulacion2.moveTo(q2,10);
  articulacion3.moveTo(q3,10);
  
  while(!((articulacion1.motor1.distanceToGo() == 0)&&(articulacion2.motor1.distanceToGo() == 0)&&(articulacion3.motor1.distanceToGo() == 0))){
    articulacion1.motor1.runSpeedToPosition();
    articulacion2.motor1.runSpeedToPosition();
    articulacion2.motor2.runSpeedToPosition();
    articulacion3.motor1.runSpeedToPosition();
  }
  currentState = WAIT_ON_COMAND;
  Serial.println("WAITING COMMAND");
}