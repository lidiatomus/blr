#include <Wire.h>
#include <QTRSensors.h>

// definire pini
#define LINE_SENSOR1_PIN A0
#define LINE_SENSOR2_PIN A1
#define OPPO_SENSOR_1_PIN 2
#define OPPO_SENSOR_2_PIN 3
#define OPPO_SENSOR_3_PIN 4
#define OPPO_SENSOR_4_PIN 5
#define OPPO_SENSOR_5_PIN 6
#define START_STOP_MODULE_PIN 7
#define MOTOR_DRIVER_1_PWM_PIN 9
#define MOTOR_DRIVER_2_PWM_PIN 10
#define MOTOR_DRIVER_1_DIR_PIN 8
#define MOTOR_DRIVER_2_DIR_PIN 11

// definire constante
#define NUM_OPPO_SENSORS 5
#define NUM_LINE_SENSORS 2
#define NUM_SENSORS_PER_OPPO 1

// creare array pt senzorii de linie
QTRSensorsAnalog lineSensors((unsigned char[]) {LINE_SENSOR1_PIN, LINE_SENSOR2_PIN}, NUM_LINE_SENSORS);

bool isRobotRunning = false;

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

  // Your main code logic goes here
}

void initializeSensors() {
  // senzori de linie
  lineSensors.init();
  lineSensors.setTimeout(2000); // Set timeout to 2000ms, asa a sugerat chat gpt ul ca sa se initializeze senzorii corect

  // senzorii de oponent
  pinMode(OPPO_SENSOR_1_PIN, INPUT);
  pinMode(OPPO_SENSOR_2_PIN, INPUT);
  pinMode(OPPO_SENSOR_3_PIN, INPUT);
  pinMode(OPPO_SENSOR_4_PIN, INPUT);
  pinMode(OPPO_SENSOR_5_PIN, INPUT);
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
  int module = digitalRead(START_STOP_MODULE_PIN);
  
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
