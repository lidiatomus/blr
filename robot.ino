#include <Wire.h>
#include <QTRSensors.h>

// definire pini
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

// definire constante
#define NUM_OPPO_SENSORS 5
#define NUM_LINE_SENSORS 2
#define NUM_SENSORS_PER_OPPO 1

//creare obiect
QTRSensors oppoSensors;
QTRSensors lineSensors;
//variabile
bool isRobotRunning = false;
bool isOpponentDetected = false;
bool isLineDetected = false;
int lastOpponentPosition = 0;
uint16_t sensorValues[NUM_SENSORS_PER_OPPO];
uint16_t lineSensorValues[NUM_LINE_SENSORS];

void setup() {
  // Initializare serial comunication
  Serial.begin(9600);

  // Initializare senzorii
  initializeSensors();

  // Initializare motoare
  initializeMotors();

  // Initializare start-stop module
  initializeStartStopModule();
  
}

void loop() {
  // Check start-stop module
  checkStartStopModule();
  updateSensorReadings();

  // Make decisions based on sensor readings
  if (isRobotRunning) {
    if (isOpponentDetected && isLineDetected) {
      attack_Opponent();
    } else if (isLineDetected && !isOpponentDetected) {
      followWhiteLine();
    } else if(!isLineDetected && isOpponentDetected) {
      runAwayFromOpponent();
    }
  }
}

void initializeSensors() {
  // Initializare senzor de linie
  unsigned char lineSensorPins[NUM_LINE_SENSORS] = {LINE_SENSOR1_PIN, LINE_SENSOR2_PIN}; 
  // Initializare senzor de oponent
  unsigned char opponentSensorPins[NUM_OPPO_SENSORS] = {OPPO_SENSOR_1_PIN, OPPO_SENSOR_2_PIN, OPPO_SENSOR_3_PIN, OPPO_SENSOR_4_PIN, OPPO_SENSOR_5_PIN};

}

void initializeMotors() {
  // drivere
  pinMode(MOTOR_DRIVER_1_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DRIVER_2_PWM_PIN, OUTPUT);
  pinMode(MOTOR_DRIVER_1_DIR_PIN, OUTPUT);
  pinMode(MOTOR_DRIVER_2_DIR_PIN, OUTPUT);
}

void initializeStartStopModule() {
  pinMode(START_STOP_MODULE_START_PIN, INPUT);
}

bool wasModulePressed = false; // ca sa verificam cand e apasat din nou start stop module (la prima apasare porneste robotul la a doua se opreste)

void checkStartStopModule() {
  int moduleState = digitalRead(START_STOP_MODULE_START_PIN);
  
  // daca primim semnal de la module pt prima oara robotul porneste
  if (moduleState == LOW && !wasModulePressed) {
    startRobot();
    isRobotRunning = true;
    wasModulePressed = true;
  } 
  // daca primim din nou semnal de la module robotul se opreste
  else if (moduleState == HIGH && wasModulePressed) {
    stopRobot();
    isRobotRunning = false;
    wasModulePressed = false;
  }
  
}

void startRobot() {
  digitalWrite(MOTOR_DRIVER_1_DIR_PIN, HIGH); 
  digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH);
  //directii
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 100); // viteza trebuie gandita pt strategie, acum e maxima
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 100); // 
  Serial.println("Robot started"); // for debugging
}

void stopRobot() {
  // Stop the motors
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 0); 
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 0); 
  Serial.println("Robot stopped"); 
}

void updateSensorReadings() {
  //citim senzorii
  oppoSensors.read(sensorValues);


  isOpponentDetected = false;
  for (int i = 0; i < NUM_OPPO_SENSORS; i++) {
    if (sensorValues[i] < LINE_THRESHOLD) {
      isOpponentDetected = true;
      lastOpponentPosition = i; // Update pozitia oponentului
      break;
    }
  }

  // Update senzorii de linie
  lineSensors.read(lineSensorValues);

  // verificam linia
  isLineDetected = (lineSensorValues[0] < LINE_THRESHOLD) || (lineSensorValues[1] < LINE_THRESHOLD);
}
void runAwayFromOpponent() {
  // Setam motoare sa fugim de oponent
  // daca e la stanga mergem la dreapta, si invers pt stanga
  if (lastOpponentPosition == 0) {
    // Opponent detected on the left
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, HIGH);  // turn right
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, LOW);   // move fwd
  } else if (lastOpponentPosition == 1 || lastOpponentPosition == 2) {
    // Opponent detected at the front (center or right)
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, LOW);   // Move backward
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, LOW);   // Move backward
  } else if (lastOpponentPosition == 3 || lastOpponentPosition == 2) {
    // Opponent detected on the left side
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, HIGH);  // Move forward
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, LOW);   // Turn right
  } else if (lastOpponentPosition == 4) {
    // Opponent detected on the right side
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, LOW);   // Turn left
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH);  // Move forward
  } else {
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, LOW);   // Turn left
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH);  // Move forward
  }
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 100);
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 100);
  delay(3000); // Wait for 3 seconds (10000 milliseconds)
  digitalWrite(MOTOR_DRIVER_1_DIR_PIN, HIGH);  // Move forward
  digitalWrite(MOTOR_DRIVER_2_DIR_PIN, LOW);   // Turn right
  updateSensorReadings(); // Update sensor readings
}
void attack_Opponent() {
  // Setam motoare sa fugim de oponent
  // daca e la stanga mergem la dreapta, si invers pt stanga
  if (lastOpponentPosition == 0) {
    // Opponent detected on the left
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, LOW);  // Move forward
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH);   // Turn LEFT
  } else if (lastOpponentPosition == 1 || lastOpponentPosition == 2) {
    // Opponent detected at the front (center or right)
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, HIGH);   // Move backward
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH);   // Move backward
  } else if (lastOpponentPosition == 3) {
    // Opponent detected on the RIGHT side
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN,LOW);  // Move forward
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH);   // Turn right
  } else if (lastOpponentPosition == 4) {
    // Opponent detected on the right side
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, HIGH);   // Turn RIGH
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, LOW);  // Move FOWARD
  }
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 150);
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 150);
  updateSensorReadings(); // Update sensor readings
}
void followWhiteLine() {
  // modificam mersul ca sa ramanem in ring
  //daca FATA -> VAL 0 , ATUNCI DAM CU SPATELE 
  if (lineSensorValues[0] > LINE_THRESHOLD) { // cat timp vede pe fata alba
    while (lineSensorValues[1] < LINE_THRESHOLD) { //si pe spate negru 180 
      digitalWrite(MOTOR_DRIVER_1_DIR_PIN, LOW);  // Move backward
      digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH); // Move backward
    }
  } else if (lineSensorValues[1] > LINE_THRESHOLD) {
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
