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

// definire constante
#define NUM_OPPO_SENSORS 5
#define NUM_LINE_SENSORS 2
#define NUM_SENSORS_PER_OPPO 1

// Creaare obiect pt senzorii de oponent
QTRSensors oppoSensors;

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
