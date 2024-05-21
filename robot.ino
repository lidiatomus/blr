#include <Wire.h>
#include <QTRSensors.h>

// Define pins
#define LINE_SENSOR1_PIN 2
#define LINE_SENSOR2_PIN 3
#define OPPO_SENSOR_1_PIN 4
#define OPPO_SENSOR_2_PIN 5
#define OPPO_SENSOR_3_PIN 6
#define OPPO_SENSOR_4_PIN 7
#define OPPO_SENSOR_5_PIN 8
#define START_STOP_MODULE_START_PIN 9
#define MOTOR_DRIVER_1_PWM_PIN 10
#define MOTOR_DRIVER_2_PWM_PIN 11
#define MOTOR_DRIVER_1_DIR_PIN 12
#define MOTOR_DRIVER_2_DIR_PIN 13
#define LINE_THRESHOLD 500

// Define constants
#define NUM_OPPO_SENSORS 5
#define NUM_LINE_SENSORS 2

// Variables
bool isRobotRunning = false;
bool isOpponentDetected = false;
bool isLineDetected = false;
int lastOpponentPosition = -1; // Initialize to -1 to indicate no detection
bool sensorValues[NUM_OPPO_SENSORS];
bool lineSensorValues[NUM_LINE_SENSORS];
bool wasModulePressed = false;

// Function declarations
void initializeSensors();
void initializeMotors();
void initializeStartStopModule();
void checkStartStopModule();
void startRobot();
void stopRobot();
void updateSensorReadings();
void runAwayFromOpponent();
void followWhiteLine();
void attack_Opponent();

void setup() {
  // Initialize serial communication
  Serial.begin(9600);

  // Initialize sensors, motors, and start-stop module
  initializeSensors();
  initializeMotors();
  initializeStartStopModule();
}

void loop() {
  // Check start-stop module
  checkStartStopModule();
  updateSensorReadings();
   printSensorValues();
  // Delay before the next reading
  delay(1000); 

  // Make decisions based on sensor readings
  if (isRobotRunning) {
    /*if (isOpponentDetected && isLineDetected) {
      attack_Opponent();
    } else if (isLineDetected && !isOpponentDetected) {
      followWhiteLine();
    } else if(!isLineDetected && isOpponentDetected) {*/
      //runAwayFromOpponent();
      attackOpponent();
    }
  }
//}

void initializeSensors() {
  // Initialize line sensors
  pinMode(LINE_SENSOR1_PIN, INPUT);
  pinMode(LINE_SENSOR2_PIN, INPUT);

  // Initialize opponent sensors
  pinMode(OPPO_SENSOR_1_PIN, INPUT);
  pinMode(OPPO_SENSOR_2_PIN, INPUT);
  pinMode(OPPO_SENSOR_3_PIN, INPUT);
  pinMode(OPPO_SENSOR_4_PIN, INPUT);
  pinMode(OPPO_SENSOR_5_PIN, INPUT);
}

void initializeMotors() {
  // Initialize motor driver pins
  pinMode(MOTOR_DRIVER_1_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DRIVER_2_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DRIVER_1_DIR_PIN, OUTPUT);
  pinMode(MOTOR_DRIVER_2_DIR_PIN, OUTPUT);
}

void initializeStartStopModule() {
  pinMode(START_STOP_MODULE_START_PIN, INPUT);
}

void checkStartStopModule() {
  int moduleState = digitalRead(START_STOP_MODULE_START_PIN);
  
  // If start-stop module is pressed for the first time, start the robot
  if (moduleState == LOW && !wasModulePressed) {
    startRobot();
    isRobotRunning = true;
    wasModulePressed = true;
  } 
  // If start-stop module is pressed again, stop the robot
  else if (moduleState == HIGH && wasModulePressed) {
    stopRobot();
    isRobotRunning = false;
    wasModulePressed = false;
  }
}

void startRobot() {
  Serial.println("Robot started");
}

void stopRobot() {
  // Stop the motors
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 0); 
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 0); 
  Serial.println("Robot stopped"); 
}

void updateSensorReadings() {
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

  // Read line sensors
  lineSensorValues[0] = digitalRead(LINE_SENSOR1_PIN);
  lineSensorValues[1] = digitalRead(LINE_SENSOR2_PIN);

  // Update line detection status
  isLineDetected = (lineSensorValues[0] == LOW) || (lineSensorValues[1] == LOW); // Assuming LOW indicates line detection
}


void followWhiteLine() {
  // Modify motion to stay in the ring
  // If front detects white (value 0), move backward
  if (lineSensorValues[0] == LOW) { // While front sees white
    while (lineSensorValues[1] != LOW) { // And back is black
      digitalWrite(MOTOR_DRIVER_1_DIR_PIN, LOW);  // Move backward
      digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH); // Move backward
      updateSensorReadings(); // Update sensor readings
    }
  } else if (lineSensorValues[1] == LOW) {
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, HIGH);
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, HIGH);
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH);
  }
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 130);
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 130);
  updateSensorReadings(); // Update sensor readings
}
void turnLeft(int duration) {
  digitalWrite(MOTOR_DRIVER_1_DIR_PIN, LOW);   // Left motor backward
  digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH);  // Right motor forward
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 250);
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 250);
  delay(duration);
  stopMotors();
}

void turnRight(int duration) {
  digitalWrite(MOTOR_DRIVER_1_DIR_PIN, HIGH);  // Left motor forward
  digitalWrite(MOTOR_DRIVER_2_DIR_PIN, LOW);   // Right motor backward
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 150);
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 150);
  delay(duration);
  stopMotors();
}

void goForward(int duration) {
  digitalWrite(MOTOR_DRIVER_1_DIR_PIN, HIGH);  // Left motor forward
  digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH);  // Right motor forward
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 250);
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 250);
  delay(duration);
  stopMotors();
}
void goBackward(int duration) {
  digitalWrite(MOTOR_DRIVER_1_DIR_PIN, LOW);  // Left motor forward
  digitalWrite(MOTOR_DRIVER_2_DIR_PIN, LOW);  // Right motor forward
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 250);
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 250);
  delay(duration);
  stopMotors();
}
void stopMotors() {
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 0);
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 0);
}
void runAwayFromOpponent() {
  Serial.println("Running away from opponent...");

  // Read the current sensor values
  updateSensorReadings();

  // Determine the action based on the current sensor readings
  if (sensorValues[0] == 1) {
    Serial.println("Opponent detected on the left. Turning right.");
    turnRight(500);  // Adjust the duration as needed
  } else if (sensorValues[1] == 1) {
    Serial.println("Opponent detected on the left front corner. Turning right.");
    turnRight(200);  // Adjust the duration as needed
  } else if (sensorValues[1] == 1 && sensorValues[2] == 1) {
    Serial.println("Opponent detected on the left front corner. Turning right.");
    goBackward(300);  // Adjust the duration as needed
    turnRight(500);
    goForward(300);
  } 
  else if (sensorValues[2] == 1) {
    Serial.println("Opponent detected at the front. Moving backward.");
    goBackward(500);  // Adjust the duration as needed
  } else if (sensorValues[3] == 1) {
    Serial.println("Opponent detected on the right front corner. Turning left.");
    turnLeft(200);  // Adjust the duration as needed
  }else if (sensorValues[3] == 1 && sensorValues[2] == 1) {
    Serial.println("Opponent detected on the right front corner. Turning left.");
    goBackward(300);  // Adjust the duration as needed
    turnLeft(200);  // Adjust the duration as needed
    goForward(300);
    
  }
   else if (sensorValues[4] == 1) {
    Serial.println("Opponent detected on the right. Turning left.");
    turnLeft(500);  // Adjust the duration as needed
  } else {
    // Fallback to last known opponent position if no sensors detect the opponent
    Serial.println("No opponent detected by sensors. Using last known position.");
    if (lastOpponentPosition == 0) {
      Serial.println("Last opponent position: left. Turning right.");
      turnRight(500);  // Adjust the duration as needed
    } else if (lastOpponentPosition == 1) {
      Serial.println("Last opponent position: left front corner. Turning right.");
      turnRight(500);  // Adjust the duration as needed
    } else if (lastOpponentPosition == 2) {
      Serial.println("Last opponent position: front. Moving backward.");
      goBackward(1000);  // Adjust the duration as needed
    } else if (lastOpponentPosition == 3) {
      Serial.println("Last opponent position: right front corner. Turning left.");
      turnLeft(500);  // Adjust the duration as needed
    } else if (lastOpponentPosition == 4) {
      Serial.println("Last opponent position: right. Turning left.");
      turnLeft(500);  // Adjust the duration as needed
    } else {
      Serial.println("Last opponent position unknown. Turning left.");
      turnLeft(500);  // Adjust the duration as needed
    }
  }

  // Continue moving forward after turning
  Serial.println("Continuing to move forward after delay.");
  goForward(1000);  // Adjust the duration as needed

  updateSensorReadings(); // Update sensor readings
  Serial.println("Updated sensor readings.");
}

void attackOpponent() {
  Serial.println("Attacking opponent...");

  // Read the current sensor values
  updateSensorReadings();

  // Determine the action based on the current sensor readings
  if (sensorValues[0] == 1) {
    Serial.println("Opponent detected on the left. Turning left.");
    turnLeft(500);  // Adjust the duration as needed
  } else if (sensorValues[1] == 1) {
    Serial.println("Opponent detected on the left front corner. Turning left.");
    turnLeft(200);  // Adjust the duration as needed
  } else if (sensorValues[1] == 1 && sensorValues[2] == 1) {
    Serial.println("Opponent detected on the left front corner. Moving forward and turning left.");
    goForward(300);  // Adjust the duration as needed
    turnLeft(500);
    goForward(300);
  } else if (sensorValues[2] == 1) {
    Serial.println("Opponent detected at the front. Moving forward.");
    goForward(500);  // Adjust the duration as needed
  } else if (sensorValues[3] == 1) {
    Serial.println("Opponent detected on the right front corner. Turning right.");
    turnRight(200);  // Adjust the duration as needed
  } else if (sensorValues[3] == 1 && sensorValues[2] == 1) {
    Serial.println("Opponent detected on the right front corner. Moving forward and turning right.");
    goForward(300);  // Adjust the duration as needed
    turnRight(200);  // Adjust the duration as needed
    goForward(300);
  } else if (sensorValues[4] == 1) {
    Serial.println("Opponent detected on the right. Turning right.");
    turnRight(500);  // Adjust the duration as needed
  } else {
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

  updateSensorReadings(); // Update sensor readings
  Serial.println("Updated sensor readings.");
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
