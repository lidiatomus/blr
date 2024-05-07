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
#define START_STOP_MODULE_PIN 9
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
    if (isOpponentDetected) {
      runAwayFromOpponent();
    } else if (isLineDetected) {
      followWhiteLine();
    } else {
      stopRobot();
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
  pinMode(START_STOP_MODULE_PIN, INPUT);
}

bool wasModulePressed = false; // ca sa verificam cand e apasat din nou start stop module (la prima apasare porneste robotul la a doua se opreste)

void checkStartStopModule() {
  int moduleState = digitalRead(START_STOP_MODULE_PIN);
  
  // daca primim semnal de la module pt prima oara robotul porneste
  if (moduleState == HIGH && !wasModulePressed) {
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
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 255); // viteza trebuie gandita pt strategie, acum e maxima
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 255); // 
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
  if (lastOpponentPosition < (NUM_OPPO_SENSORS / 2)) {
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, HIGH);
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH);
  } else {
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, LOW);
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, LOW);
  }
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 200);
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 200);
}

void followWhiteLine() {
  // modificam mersul ca sa ramanem in ring
  //daca stanga atunci dreapta si invers, daca nimic atunci mergem in fata
  if (lineSensorValues[0] < LINE_THRESHOLD) {
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, LOW);
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH);
  } else if (lineSensorValues[1] < LINE_THRESHOLD) {
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, HIGH);
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, LOW);
  } else {
    digitalWrite(MOTOR_DRIVER_1_DIR_PIN, HIGH);
    digitalWrite(MOTOR_DRIVER_2_DIR_PIN, HIGH);
  }
  analogWrite(MOTOR_DRIVER_1_PWM_PIN, 200);
  analogWrite(MOTOR_DRIVER_2_PWM_PIN, 200);
}
