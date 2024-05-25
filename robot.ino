 #include <Wire.h>
#include "CytronMotorDriver.h"

// Define pins
#define LINE_SENSOR1_PIN 2
#define LINE_SENSOR2_PIN 3
#define OPPO_SENSOR_1_PIN 4
#define OPPO_SENSOR_2_PIN 5
#define OPPO_SENSOR_3_PIN 6
#define OPPO_SENSOR_4_PIN 7
#define OPPO_SENSOR_5_PIN 8
#define START_STOP_MODULE_START_PIN 9
#define MOTOR_DRIVER_1_PWM_PIN 10 //analog
#define MOTOR_DRIVER_2_PWM_PIN 11
#define MOTOR_DRIVER_1_DIR_PIN 12 //digital
#define MOTOR_DRIVER_2_DIR_PIN 13
#define LINE_THRESHOLD 1000// de hotarat

// Define constants
#define NUM_OPPO_SENSORS 5
#define NUM_LINE_SENSORS 2 

// Variables
bool isRobotRunning = false;
bool isOpponentDetected = false;
bool isLineDetected = false;
int lastOpponentPosition = -1; // Initialize to -1 to indicate no detection
bool sensorValues[NUM_OPPO_SENSORS];
unsigned long lineSensorValues[NUM_LINE_SENSORS];
bool wasModulePressed = false;
unsigned long sensorTime;
// Function declarations
void initializeOppoSensors();
void initializeMotors();
void initializeStartStopModule();
void checkStartStopModule();
void startRobot();
void stopRobot();
void updateOppoSensorReadings();
void lineSensor();
void runAwayFromOpponent();
void followWhiteLine();
void attackOpponent();
void lastPosition();
CytronMD motor1(PWM_DIR,10 , 12);  // PWM 1 = Pin 3, DIR 1 = Pin 4.
CytronMD motor2(PWM_DIR, 11, 13); // PWM 2 = Pin 9, DIR 2 = Pin 10.
void setup() {
  
  Serial.begin(9600);
  delay(3000); 
  // Initialize sensors, motors, and start-stop module
  initializeOppoSensors();
  //initializeMotors();
  initializeStartStopModule();
}

void loop() {
  int moduleState = digitalRead(START_STOP_MODULE_START_PIN);
  
  // If start-stop module is pressed for the first time, start the robot
  if (moduleState == 1) {
    startRobot();
    isRobotRunning = true;
     Serial.println("Robot started"); 
  } 
  // If start-stop module is pressed again, stop the robot
  else if (moduleState == 0 ) {
    stopRobot();
    isRobotRunning = false;
     Serial.println("Robot stopped"); 
  }
   if (isRobotRunning){
     lineSensor();
    updateOppoSensorReadings();
    //printSensorValues();
   attackOpponent();
    followWhiteLine();}
    //delay(3000);
    //stopRobot();
    //delay(100);
  // Make decisions based on sensor readings
  /*if (isRobotRunning) {
    if (isOpponentDetected && isLineDetected) {
      attackOpponent();
      //followWhiteLine();
    } else if (isLineDetected && !isOpponentDetected) {
      followWhiteLine();
    } else if(!isLineDetected && isOpponentDetected) {
      runAwayFromOpponent();
      followWhiteLine();
    }
  }*/
   //attackOpponent();
}

void initializeOppoSensors() {
  
  pinMode(OPPO_SENSOR_1_PIN, INPUT);
  pinMode(OPPO_SENSOR_2_PIN, INPUT);
  pinMode(OPPO_SENSOR_3_PIN, INPUT);
  pinMode(OPPO_SENSOR_4_PIN, INPUT);
  pinMode(OPPO_SENSOR_5_PIN, INPUT);
}


void initializeStartStopModule() {
  pinMode(START_STOP_MODULE_START_PIN, INPUT);
}
/*
void checkStartStopModule() {
  int moduleState = digitalRead(START_STOP_MODULE_START_PIN);
  
  // If start-stop module is pressed for the first time, start the robot
  if (moduleState == 1) {
    startRobot();
    isRobotRunning = true;
  } 
  // If start-stop module is pressed again, stop the robot
  else if (moduleState == 0 ) {
    stopRobot();
    isRobotRunning = false;
  }
}
*/
void startRobot() {
  motor1.setSpeed(100);   // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(100);  // Motor 2 runs backward at 50% speed.
  Serial.println("Robot started");
}

void stopRobot() {
  // Stop the motors
 motor1.setSpeed(0);     // Motor 1 stops.
  motor2.setSpeed(0);     // Motor 2 stops.
  Serial.println("Robot stopped"); 
}

void updateOppoSensorReadings() {
  // Read opponent sensors
  sensorValues[0] = digitalRead(OPPO_SENSOR_1_PIN);
  sensorValues[1] = digitalRead(OPPO_SENSOR_2_PIN);
  sensorValues[2] = digitalRead(OPPO_SENSOR_3_PIN);
  sensorValues[3] = digitalRead(OPPO_SENSOR_4_PIN);
  sensorValues[4] = digitalRead(OPPO_SENSOR_5_PIN);

  // Update opponent detection status
  isOpponentDetected = false;
  for (int i = 0; i < NUM_OPPO_SENSORS; i++) {
    if (sensorValues[i] == 1) { 
      isOpponentDetected = true;
      lastOpponentPosition = i;
    }
  }

}
void lineSensor() {
  // Measure the discharge time for the first line sensor
  pinMode(LINE_SENSOR1_PIN, OUTPUT);
  digitalWrite(LINE_SENSOR1_PIN, HIGH);
  delayMicroseconds(12);
  pinMode(LINE_SENSOR1_PIN, INPUT);
  sensorTime = micros();
  while (digitalRead(LINE_SENSOR1_PIN)) {}
  lineSensorValues[0] = micros() - sensorTime;

  // Measure the discharge time for the second line sensor
  pinMode(LINE_SENSOR2_PIN, OUTPUT);
  digitalWrite(LINE_SENSOR2_PIN, HIGH);
  delayMicroseconds(12);
  pinMode(LINE_SENSOR2_PIN, INPUT);
  sensorTime = micros();
  while (digitalRead(LINE_SENSOR2_PIN)) {}
  lineSensorValues[1] = micros() - sensorTime;

  // Print the sensor values for debugging
  Serial.print("Line Sensor 1: ");
  Serial.println(lineSensorValues[0]);
  Serial.print("Line Sensor 2: ");
  Serial.println(lineSensorValues[1]);

  // Determine if the line is detected
  isLineDetected = (lineSensorValues[0] < LINE_THRESHOLD) || (lineSensorValues[1] < LINE_THRESHOLD);
  if(isLineDetected == true) 
  Serial.println("line detected ");
}

void followWhiteLine() {
  // Modify motion to stay in the ring
  // If front detects white (value 0), move backward
  if (lineSensorValues[0] < LINE_THRESHOLD) { // While stanga sees white 
      turnRight(1000) ;
    }
   else if (lineSensorValues[1] <LINE_THRESHOLD) {
    turnLeft(1000);
  } else if(lineSensorValues[0] < LINE_THRESHOLD && lineSensorValues[1] < LINE_THRESHOLD)
  {
  goBackward(2000);}
  else {
    motor1.setSpeed(50);   // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(50);
  }
  lineSensor(); // Update line sensor readings
}

void turnLeft(int duration) {
  motor1.setSpeed(-200);   // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(200); 
  delay(duration);
}

void turnRight(int duration) {
  motor1.setSpeed(-200);   // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(200); 
  delay(duration);
}

void goForward(int duration) {
  motor1.setSpeed(100);   // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(100); 
  delay(duration);
}
void goBackward(int duration) {
  motor1.setSpeed(-100);   // Motor 1 runs forward at 50% speed.
  motor2.setSpeed(-100); 
  delay(duration);
}

void runAwayFromOpponent() {
  Serial.println("Running away from opponent...");

  // Read the current sensor values
  updateOppoSensorReadings();

  // Determine the action based on the current sensor readings
  if (sensorValues[0] == 1) {
    Serial.println("Opponent detected on the left. Turning right.");
    turnRight(300); } // Adjust the duration as needed*/
    if (sensorValues[1] == 1) {
    Serial.println("Opponent detected on the left front corner. Turning right.");
    turnRight(100);  // Adjust the duration as needed
  } else if (sensorValues[1] == 1 && sensorValues[2] == 1) {
    Serial.println("Opponent detected on the left front corner. Turning right.");
    goBackward(200);  // Adjust the duration as needed
    turnRight(200);
    goForward(200);
  } 
  else if (sensorValues[2] == 1) {
    Serial.println("Opponent detected at the front. Moving backward.");
    goBackward(200);  // Adjust the duration as needed
  } else if (sensorValues[3] == 1) {
    Serial.println("Opponent detected on the right front corner. Turning left.");
    turnLeft(200);  // Adjust the duration as needed
  }else if (sensorValues[3] == 1 && sensorValues[2] == 1) {
    Serial.println("Opponent detected on the right front corner. Turning left.");
    goBackward(100);  // Adjust the duration as needed
    turnLeft(100);  // Adjust the duration as needed
    goForward(100);
    
  }
   else if (sensorValues[4] == 1) {
    Serial.println("Opponent detected on the right. Turning left.");
    turnLeft(200);  // Adjust the duration as needed
  } 
  else followWhiteLine();
 
  updateOppoSensorReadings(); // Update sensor readings
  Serial.println("Updated sensor readings.");
}

void attackOpponent() {
  Serial.println("Attacking opponent...");

  // Read the current sensor values
  updateOppoSensorReadings();

  // Determine the action based on the current sensor readings
  /*if (sensorValues[0] == 1) {
    Serial.println("Opponent detected on the left. Turning left.");
    turnLeft(100);  // Adjust the duration as needed
    goForward(500);  // Adjust the duration as needed

  } else*/ if (sensorValues[1] == 1) {
    Serial.println("Opponent detected on the left front corner. Turning left.");
    turnLeft(50);
    goForward(600);  // Adjust the duration as needed
  } else if (sensorValues[1] == 1 && sensorValues[2] == 1) {
    Serial.println("Opponent detected on the left front corner. Moving forward.");
    goForward(500);  // Adjust the duration as needed
  } else if (sensorValues[2] == 1) {
    Serial.println("Opponent detected at the front. Moving forward.");
    goForward(600);  // Adjust the duration as needed
  } else if (sensorValues[3] == 1) {
    Serial.println("Opponent detected on the right front corner. Turning right.");
    turnRight(100);  // Adjust the duration as needed
    goForward(500);
  }else if (sensorValues[3] == 1 && sensorValues[2] == 1 && sensorValues[1] == 1) {
    Serial.println("Opponent detected in front.");
    goForward(400);  // Adjust the duration as needed
  } 
  else if (sensorValues[3] == 1 && sensorValues[2] == 1) {
    Serial.println("Opponent detected on the right front corner. Moving forward and turning right.");
    goForward(400);  // Adjust the duration as needed
  } else /*if (sensorValues[4] == 1) {
    Serial.println("Opponent detected on the right. Turning right.");
    turnRight(500);  // Adjust the duration as needed
  } */
  followWhiteLine();
  updateOppoSensorReadings(); // Update sensor readings
  Serial.println("Updated sensor readings.");
}
void lastPosition()
  {
    // Fallback to last known opponent position if no sensors detect the opponent
    Serial.println("No opponent detected by sensors. Using last known position.");
    if (lastOpponentPosition == 0) {
      Serial.println("Last opponent position: left. Turning left.");
      turnLeft(500);  // Adjust the duration as needed
    } else if (lastOpponentPosition == 1) {
      Serial.println("Last opponent position: left front corner. Turning left.");
      turnLeft(500);  // Adjust the duration as needed
    } else if (lastOpponentPosition == 2) {
      Serial.println("Last opponent position: front. Moving forward.");
      goForward(1000);  // Adjust the duration as needed
    } else if (lastOpponentPosition == 3) {
      Serial.println("Last opponent position: right front corner. Turning right.");
      turnRight(500);  // Adjust the duration as needed
    } else if (lastOpponentPosition == 4) {
      Serial.println("Last opponent position: right. Turning right.");
      turnRight(500);  // Adjust the duration as needed
    } else {
      Serial.println("Last opponent position unknown. Moving forward.");
      goForward(1000);  // Adjust the duration as needed
    }
  }


void printSensorValues() {
  // Print the sensor values to the Serial Monitor
  Serial.print("Opponent Sensor Values: ");
  for (int i = 0; i < NUM_OPPO_SENSORS; i++) {
    Serial.print(sensorValues[i]);
    if (i < NUM_OPPO_SENSORS - 1) {
      Serial.print(", ");
    }
  }
  Serial.println();
}