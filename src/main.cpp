#include <Arduino.h>

//DEFINES
#define DEBUG 1

//PIN DEFINITIONS
const int PIN_LED = LED_BUILTIN;           //D8 | Digital 8 | GPIO 14
const int PIN_BUTTON = 9;        //D9 | Digital 9 | GPIO 15 --???????? TODO: Revisar todos los pines

//MOTOR RIGHT
const int PIN_MOTOR_R_PWM = 5;   //D5 | Digital 5 | GPIO 11
const int PIN_MOTOR_R_1 = 6;     //D6 | Digital 6 | GPIO 12
const int PIN_MOTOR_R_2 = 7;     //D7 | Digital 7 | GPIO 13

//MOTOR LEFT
const int PIN_MOTOR_L_PWM = 3;    //D3 | Digital 3 | GPIO 5
const int PIN_MOTOR_L_1 = 2;      //D2 | Digital 2 | GPIO 4
const int PIN_MOTOR_L_2 = 4;      //D4 | Digital 4 | GPIO 6

//SENSOR PINS
const int PIN_SENSOR_0 = 11;
const int PIN_SENSOR_1 = 14;
const int PIN_SENSOR_2 = 15;
const int PIN_SENSOR_3 = 16;
const int PIN_SENSOR_4 = 17;
const int PIN_SENSOR_5 = 18;
const int PIN_SENSOR_6 = 19;
const int PIN_SENSOR_7 = 12;

const int CANT_ANALOG_SENSORS = 6;
const int CANT_DIGITAL_SENSORS = 2;

const int CANT_ALL_SENSORS = CANT_ANALOG_SENSORS + CANT_DIGITAL_SENSORS;


const int PINS_ANALOG_SENSORS[CANT_ANALOG_SENSORS] = {PIN_SENSOR_1, PIN_SENSOR_2, PIN_SENSOR_3, PIN_SENSOR_4, PIN_SENSOR_5, PIN_SENSOR_6};
const int PINS_DIGITAL_SENSORS[CANT_DIGITAL_SENSORS] = {PIN_SENSOR_0, PIN_SENSOR_7};


//GLOBAL CONSTANTS
const int ANALOG_SENSOR_THRESHOLD = 1024;
const int ANALOG_SENSOR_MAX = 1024;
const int MOTORS_MAX_PWM_VALUE = 1024;

const int DELAY_MS_MAIN_LOOP = 100;

const int CANT_LONGPRESS_LEN = 600 / DELAY_MS_MAIN_LOOP;  // 2.5 seconds
const int CANT_SHORTPRESS_LEN = 100 / DELAY_MS_MAIN_LOOP;   // 0.1 seconds
// const int CANT_WAIT_TO_ACTION_LEN = 200 / DELAY_MS_MAIN_LOOP;   // 0.2 seconds

const int SPEED_BASE = 50;
const int SPEED_INCREMENT = 30;
const int SPEED_INCREMENT_SOFT = 10;

//STRUCTURES
struct MotorsSpeeds {
  int leftSpeed;
  int rightSpeed;
};

struct SensorsData {
  int analogSensorValues[CANT_ANALOG_SENSORS];
  int digitalSensorValues[CANT_ALL_SENSORS];
};

//ENUMS
enum events { EV_NONE = 0, EV_SHORTPRESS = 1, EV_LONGPRESS = 2};
enum state { STATE_SETUP= -1, STATE_STOP = 0, STATE_CALIBRATION = 1, STATE_RUNNING = 2};

//GLOBAL VARIABLES

int ledState = 0;
state currentState = STATE_SETUP;


//FUNCTIONS
void calibration();
SensorsData readSensorsValues();
void printSensorsValues(SensorsData sensorData);
MotorsSpeeds calculateMotorsSpeeds(SensorsData sensorData);
void printMotorsSpeeds(MotorsSpeeds motorSpeeds);
void applySpeedsToMotors(MotorsSpeeds motorSpeeds);
events handle_button();
void handle_led(events event);


void handle_led(state currentState) {
  static int count = 0;
  static int ledState = 0;
  // 0.1 seconds

  switch (currentState)
  {
    case STATE_SETUP:
    {
      if (count > 100 / DELAY_MS_MAIN_LOOP){
        ledState = !ledState;
        count = 0;
      } else {
        ++count;
        ledState = !ledState;
      }
      break;
    }
    case STATE_STOP:
    {
      ledState = HIGH;
      break;
    }
    case STATE_CALIBRATION:
    {
      if (count > 1000 / DELAY_MS_MAIN_LOOP){
        ledState = !ledState;
        count = 0;
      } else {
        ++count;
        ledState = !ledState;
      }
      break;
    }
    case STATE_RUNNING:
    {
      if (count > 1500 / DELAY_MS_MAIN_LOOP){
        ledState = !ledState;
        count = 0;
      } else {
        ++count;
        ledState = !ledState;
      }
      break;
    }
  }

  digitalWrite(PIN_LED, ledState);
}


void setup() {

  //initialize the serial communication
  Serial.begin(9600);
  Serial.println("STARTING THE PROGRAM");

  //initialize the LED pin as an output
  pinMode(PIN_LED, OUTPUT);

  //initialize the button pin as an input
  pinMode(PIN_BUTTON, INPUT_PULLUP);

  //initialize 6 analog inputs for sensors
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++) {
    pinMode(PINS_ANALOG_SENSORS[i], INPUT);
  }

  //initialize 2 digital inputs for sensors
  for (int i = 0; i < CANT_DIGITAL_SENSORS; i++) {
    pinMode(PINS_DIGITAL_SENSORS[i], INPUT);
  }

  //initialize the 3 outputs for each motors
  pinMode(PIN_MOTOR_L_PWM, OUTPUT);
  pinMode(PIN_MOTOR_L_1, OUTPUT);
  pinMode(PIN_MOTOR_L_2, OUTPUT);

  pinMode(PIN_MOTOR_R_PWM, OUTPUT);
  pinMode(PIN_MOTOR_R_1, OUTPUT);
  pinMode(PIN_MOTOR_R_2, OUTPUT);

  //print the message to the serial monitor
  Serial.println("INITIALIZATION COMPLETED");

  currentState = STATE_STOP;
}

void loop() {

  events event = handle_button();

  handle_led(currentState);

  //print for serial monitor the event
  if(DEBUG == 1){
    Serial.print("Evn: "); // event
    Serial.print(event);
    Serial.print(" | Btn: "); // button
    Serial.print(digitalRead(PIN_BUTTON));
    Serial.print(" | Sta: "); // state
    Serial.println(currentState);
  }

  switch (currentState) {

    case STATE_STOP:
    {
      if (event == EV_SHORTPRESS) {
        currentState = STATE_RUNNING;
      } 
      break;
    }
    
    case STATE_RUNNING:
    {

      if (event == EV_SHORTPRESS) {
        currentState = STATE_STOP;
        break;
      }

      SensorsData sensorData = readSensorsValues();
      
      if(DEBUG == 1) {
        printSensorsValues(sensorData);
      } 
    
      MotorsSpeeds motorsSpeeds = calculateMotorsSpeeds(sensorData);

      if(DEBUG == 1) {
        printMotorsSpeeds(motorsSpeeds);
      } 

      applySpeedsToMotors(motorsSpeeds);

      // if(DEBUG == 1){
      //   Serial.println("==============================================================");
      // }

      break;
    }
    
    default:
    {
      break;
    }
  }

  delay(DELAY_MS_MAIN_LOOP);
}


SensorsData readSensorsValues() {

  SensorsData sensorData = {.analogSensorValues = {0, 0, 0, 0, 0, 0}, .digitalSensorValues = {0, 0}};
  
  //analog read
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++) {
    sensorData.analogSensorValues[i] = analogRead(PINS_ANALOG_SENSORS[i]);
  }

  //digital read
  sensorData.digitalSensorValues[0] = digitalRead(PINS_DIGITAL_SENSORS[0]);

  //----analog to digital conversion
  for (int i = 0; i < CANT_ANALOG_SENSORS; i++) {
    sensorData.digitalSensorValues[i + 1] = sensorData.analogSensorValues[i] > (ANALOG_SENSOR_THRESHOLD / 2) ? 1 : 0;
    }

  sensorData.digitalSensorValues[CANT_ALL_SENSORS - 1] = digitalRead(PINS_DIGITAL_SENSORS[1]);

  //return the sensor data
  return sensorData;
}

void printSensorsValues(SensorsData sensorData) {
  //print the values of the sensors to the serial monitor
  Serial.print("AV: ");
  for (int i = 0; i < 6; i++) {
    Serial.print(sensorData.analogSensorValues[i]);
    Serial.print(" : ");
  }
  Serial.println("");

  Serial.print("DV: ");
  for (int i = 0; i < 8; i++) {
    Serial.print(sensorData.digitalSensorValues[i] == 1 ? " [" + String(i + 1) + "]" : " ___");
  }
  Serial.println("");

}

MotorsSpeeds calculateMotorsSpeeds(SensorsData sensorData) {

  MotorsSpeeds motorsSpeeds = {.leftSpeed = 0, .rightSpeed = 0}; 
  int weights[] = {5, 3, 2, 1, 1, 2, 3, 5};
  
  int maxLeftDetections = 0;
  for (int i = 0; i < CANT_DIGITAL_SENSORS / 2; i++) {
    if (sensorData.digitalSensorValues[i] == 1 && weights[i] > maxLeftDetections){
      maxLeftDetections = weights[i];
    }
  }

  int maxRightDetections = 0;
  for (int i = CANT_DIGITAL_SENSORS / 2; i < CANT_DIGITAL_SENSORS; i++) {
    if (sensorData.digitalSensorValues[i] == 1 && weights[i] > maxRightDetections){
      maxRightDetections = weights[i];
    }
  }
  
  //El output va a ir tomando valores positivos y negativos 
  motorsSpeeds.leftSpeed = SPEED_BASE + SPEED_INCREMENT * maxRightDetections; // Motor izquierdo + Error derecho
  motorsSpeeds.rightSpeed = SPEED_BASE + SPEED_INCREMENT * maxLeftDetections; // Motor derecho + Error izquierdo

  Serial.print("NCons L = ");
  Serial.print(motorsSpeeds.leftSpeed);
  Serial.print(" | R = ");
  Serial.println(motorsSpeeds.rightSpeed);

  // Asegurarse de que las velocidades no excedan los límites
  motorsSpeeds.leftSpeed = constrain(motorsSpeeds.leftSpeed, 0, MOTORS_MAX_PWM_VALUE);
  motorsSpeeds.rightSpeed = constrain(motorsSpeeds.rightSpeed, 0, MOTORS_MAX_PWM_VALUE);

  Serial.print("WCons L = ");
  Serial.print(motorsSpeeds.leftSpeed);
  Serial.print(" | R = ");
  Serial.println(motorsSpeeds.rightSpeed);

  return motorsSpeeds;
}

void printMotorsSpeeds(MotorsSpeeds motorSpeeds) {
  //print the motor speeds to the serial monitor
  Serial.print("Calc L = ");
  Serial.print(motorSpeeds.leftSpeed);
  Serial.print(" | R = ");
  Serial.println(motorSpeeds.rightSpeed);
}

void applySpeedsToMotors(MotorsSpeeds motorSpeeds) {
  //apply the motor speeds to the motors

  static int actualLeftPWMSpeed = 0;
  static int actualRightPWMSpeed = 0;
  static int actualLeftDirection = 0;
  static int actualRightDirection = 0;

  int desiredLeftSpeed = abs(motorSpeeds.leftSpeed);
  int desiredRightSpeed = abs(motorSpeeds.rightSpeed);
  int desiredLeftDirection = motorSpeeds.leftSpeed > 0 ? 1 : motorSpeeds.leftSpeed < 0 ? -1 : 0; // load direction of the motors (forward or zero or backward)
  int desiredRightDirection = motorSpeeds.rightSpeed > 0 ? 1 : motorSpeeds.rightSpeed < 0 ? -1 : 0;

  if(DEBUG == 1){
    //print the desired speeds and directions
    Serial.print("Desired L = ");
    Serial.print(desiredLeftSpeed);
    Serial.print(" | R = ");
    Serial.println(desiredRightSpeed);
    Serial.print("Dir L = ");
    Serial.print(desiredLeftDirection);
    Serial.print(" | R = ");
    Serial.println(desiredRightDirection);

    //print the actual speeds and directions
    Serial.print("Actual L = ");
    Serial.print(actualLeftPWMSpeed);
    Serial.print(" | R = ");
    Serial.println(actualRightPWMSpeed);
    Serial.print("Dir L = ");
    Serial.print(actualLeftDirection);
    Serial.print(" | R = ");
    Serial.println(actualRightDirection);
  }
  

  // Adjust softly the speed of the motors
  if (desiredRightSpeed > actualRightPWMSpeed) {
    actualRightPWMSpeed += SPEED_INCREMENT_SOFT;
  } else if (desiredRightSpeed < actualRightPWMSpeed) {
    actualRightPWMSpeed += SPEED_INCREMENT_SOFT;
  }

  if (desiredLeftSpeed > actualLeftPWMSpeed) {
    actualLeftPWMSpeed += SPEED_INCREMENT_SOFT;
  } else if (desiredLeftSpeed < actualLeftPWMSpeed) {
    actualLeftPWMSpeed += SPEED_INCREMENT_SOFT;
  }
  
  // Motor izquierdo
  if (actualLeftDirection > 0) {
    digitalWrite(PIN_MOTOR_L_1, HIGH);
    digitalWrite(PIN_MOTOR_L_2, LOW);
  } else if (actualLeftDirection < 0) {
    digitalWrite(PIN_MOTOR_L_1, LOW);
    digitalWrite(PIN_MOTOR_L_2, HIGH);
  } else {
    digitalWrite(PIN_MOTOR_L_1, LOW);
    digitalWrite(PIN_MOTOR_L_2, LOW);
  }

  // Motor derecho
  if (actualRightDirection > 0) {
    digitalWrite(PIN_MOTOR_R_1, LOW);
    digitalWrite(PIN_MOTOR_R_2, HIGH);
  } else if (actualRightDirection < 0) {
    digitalWrite(PIN_MOTOR_R_1, HIGH);
    digitalWrite(PIN_MOTOR_R_2, LOW);
  } else {
    digitalWrite(PIN_MOTOR_R_1, LOW);
    digitalWrite(PIN_MOTOR_R_2, LOW);
  }

  // Set the PWM values
  analogWrite(PIN_MOTOR_L_PWM, actualLeftPWMSpeed);
  analogWrite(PIN_MOTOR_R_PWM, actualRightPWMSpeed);

}

events handle_button()
{
  static int button_pressed_counter = 0;
  static int button_not_pressed_counter = 0;
  events event = EV_NONE;
  int button_now_pressed = !digitalRead(PIN_BUTTON); // pin low -> pressed

  if (button_now_pressed){
    ++button_pressed_counter;
    button_not_pressed_counter = 0;
  }    
  else{
    ++button_not_pressed_counter;
  }

  if (button_not_pressed_counter >= CANT_SHORTPRESS_LEN){
    if (button_pressed_counter >= CANT_LONGPRESS_LEN){
    event = EV_LONGPRESS;
    }
    else if (button_pressed_counter >= CANT_SHORTPRESS_LEN){
      event = EV_SHORTPRESS;
    }
    button_pressed_counter = 0;
  }
  return event;
}
