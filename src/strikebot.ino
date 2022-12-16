// Strikebot v0 Arduino Uno Code April 2022
// Latest version
// Check pin numbers and PWM pin requirements before testing
// PIN 2 AND 3 ARE TX/RX FOR BLE MODULE WHEN USING DABBLE GAMEPAD

// Default Arduino libraries
#include <Wire.h>
#include <Servo.h>

// #include <SoftwareSerial.h>

// Dabble gamepad imports
#define CUSTOM_SETTINGS
#define INCLUDE_GAMEPAD_MODULE
#include <Dabble.h>

// Libraries that need to be installed
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include "CytronMotorDriver.h"

// PIN INITIALIZATION
// TX/RX pin numbers need to be 2 and 3 for Dabble Gamepad module to work 
#define BLE_TX_PIN 2
#define BLE_RX_PIN 3

#define SERVO1_PIN 5
#define SERVO2_PIN 6

#define DCMOTOR1_PWM_PIN 11
#define DCMOTOR2_PWM_PIN 11
#define DCMOTOR1_DIR_PIN 12
#define DCMOTOR2_DIR_PIN 13

// Max angle for turning when in manual control mode
#define MAX_MOTOR_SPEED 160
#define MAX_SERVO_ANGLE 30
// Value of range for which the input should ignore the servo command
#define IGNORE_TOLERANCE 20


// Initializing driver objects
Servo servo1;
Servo servo2;
Adafruit_MPU6050 mpu;
CytronMD motor1(PWM_DIR, DCMOTOR1_PWM_PIN, DCMOTOR1_DIR_PIN);
CytronMD motor2(PWM_DIR, DCMOTOR2_PWM_PIN, DCMOTOR2_DIR_PIN);

// command = {motor_dir, motor_speed, servo_dir, servo_angle, damping, stop}
//short [] command = {0, 0, 0, 0, 0, 1};

bool isSquarePressed, isCirclePressed, isCrossPressed, isTrianglePressed, isStartPressed, isSelectPressed;
int inputAngle, inputRadius;
float xInput, yInput;

int motorDirection, motorSpeed, servoAngle;
bool isDampingActive = true;
bool emergencyStop = false;

// int servo_pin = 5;
int error,error_i,error_d, prev_err=0, angle, kp=1, ki=0, kd=0.1;
int dt = 10;

float roll, pitch, yaw;
float accelX, accelY, accelZ;

#define LED_PIN 8

void setup(void) {
  pinMode(LED_PIN, OUTPUT);
  Serial.begin(9600);
  Dabble.begin(9600);
  while (!Serial)
    delay(10); // will pause Zero, Leonardo, etc until serial console opens

  Serial.println("Adafruit MPU6050 test!");
  delay(1000);
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");
  //mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  //mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  delay(10);
  servo1.attach(SERVO1_PIN);
  servo2.attach(SERVO2_PIN);
}

void loop() {

  /* Get new sensor events with the readings */

  
  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  accelX = a.acceleration.x;
  accelY = a.acceleration.y;
  accelZ = a.acceleration.z;

  pitch = 180 * atan2(accelX, sqrt(accelY*accelY + accelZ*accelZ))/PI;
//  roll = 180 * atan2(accelY, sqrt(accelX*accelX + accelZ*accelZ))/PI;
  

  processBLEInput();
  
  // Stop motors with active damping on when emergency stop is called/
  if(emergencyStop){
    motor1.setSpeed(0);
    motor2.setSpeed(0);
  } else {
    motor1.setSpeed(motorSpeed);
    motor2.setSpeed(180 - motorSpeed);
  }

  if(isDampingActive){
    // Runs if active damping is ON
    error = pitch;
    error_d = (error-prev_err)/dt;
    prev_err = error;
    // Write the control value on both servos
    servo1.write(90 - (kp*error+kd*error_d));
    servo2.write(90 + (kp*error+kd*error_d));
    //Serial.print("Pitch: "); Serial.print(pitch);
    //Serial.print("\tError_D: "); Serial.print(error_d);
    //Serial.print("\tError: "); Serial.print(error);
    //Serial.print("\tPrev_Err: "); Serial.println(prev_err);
    Serial.print("90 - (kp*error+kd*error_d): "); Serial.println(90 - (kp*error+kd*error_d));
  }
  else{
    // For turning on user input
    servo1.write(servoAngle);
    servo2.write(servoAngle);

  Serial.print("MotorSpeed: ");
  Serial.print(motorSpeed);
  Serial.print("\tServoAngle: ");
  Serial.print(servoAngle);
  Serial.print("\t");
//  Serial.print("\tDamping: ");/
  Serial.print(isDampingActive);
  Serial.print("\t");
//  Serial.print("\tEmergencySto/p");
  Serial.print(emergencyStop);
  Serial.println("");
  }

  delay(40);
}

void processBLEInput(){
  
  Dabble.processInput();
  isSelectPressed = GamePad.isSelectPressed();
  isStartPressed = GamePad.isStartPressed();

  isSquarePressed = GamePad.isSquarePressed();
  isCirclePressed = GamePad.isCirclePressed();
  isCrossPressed = GamePad.isCrossPressed();
  isTrianglePressed = GamePad.isTrianglePressed();

  inputAngle = GamePad.getAngle();
  inputRadius = GamePad.getModuleId();
  xInput = GamePad.getXaxisData();
  yInput = GamePad.getYaxisData();

  motorSpeed = map(yInput, -6, +6, -MAX_MOTOR_SPEED, MAX_MOTOR_SPEED);
  servoAngle = map(xInput, -6, +6, -MAX_SERVO_ANGLE, MAX_SERVO_ANGLE);

  if (((inputAngle < (90 + IGNORE_TOLERANCE)) && (inputAngle > (90 - IGNORE_TOLERANCE))) | \
  ((inputAngle < (270 + IGNORE_TOLERANCE)) && (inputAngle > (270 - IGNORE_TOLERANCE)))){
    servoAngle = 0;
  }

  if (((inputAngle < (180 + IGNORE_TOLERANCE)) && (inputAngle > (180 - IGNORE_TOLERANCE))) | \
  ((inputAngle < IGNORE_TOLERANCE) && (inputAngle > - IGNORE_TOLERANCE))){
    motorSpeed = 0;
  }  

  // Configure the sign to match with right or left
  servoAngle = 90 + servoAngle;

  if (isTrianglePressed){
    digitalWrite(LED_PIN, HIGH);
    isDampingActive = true;
  }

  if(isCrossPressed){
    digitalWrite(LED_PIN, LOW);
    isDampingActive = false;
  }

  if(isSelectPressed){
    // Activated Emergency Stop Mode
    emergencyStop = true;
    isDampingActive = true;
    motorSpeed = 0;
    
  }

  if(isStartPressed){
    // Deactivates emergency stop mode
    emergencyStop = false;
    servoAngle = 90;
    motorSpeed = 0;
    isDampingActive = true;
  }

  if(emergencyStop){
    motorSpeed = 0.0;
  }
  
  // command = {motor_dir, motor_speed, servo_dir, servo_angle, damping, stop}

  
}
