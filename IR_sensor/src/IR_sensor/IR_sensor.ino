#include "Arduino.h"
#include <SPI.h>
#include <Wire.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <Adafruit_MotorShield.h>

// Create the motor shield object with the default I2C address
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

// Select which 'port' M1, M2, M3 or M4.
Adafruit_DCMotor *motorL = AFMS.getMotor(4);
Adafruit_DCMotor *motorR = AFMS.getMotor(3);

// Define pins
const int IR_L_READ = A2;
const int IR_M_READ = A1;
const int IR_R_READ = A0;

const int IR_L_OUT = 5;
const int IR_M_OUT = 4;
const int IR_R_OUT = 3;

const int speed = 25;
const int turnspeed = 40;

// Define threshold value for black tape reading
int THRESHOLD = 700; // if over 700, then on black tape

int getReading(int D_pin, int A_pin) {
  // IR LED on
  digitalWrite(D_pin, HIGH);
  // Read phototransistor voltage
  int v_on = analogRead(A_pin);
  // IR LED off
  digitalWrite(D_pin, LOW);
  // Read phototransistor voltage
  int v_off = analogRead(A_pin);
  // Subtract the ambient noise

  int value = v_on - v_off;
  return value;
 }

void goLeftInPlace() {
    motorL->run(BACKWARD);
    motorR->run(FORWARD);
    motorL->setSpeed(0);
    motorR->setSpeed(0);
    delay(5);
    motorL->setSpeed(turnspeed);
    motorR->setSpeed(turnspeed);
 }

void goRightInPlace() {
    motorR->run(BACKWARD);
    motorL->run(FORWARD);
    motorL->setSpeed(0);
    motorR->setSpeed(0);
    delay(5);
    motorL->setSpeed(turnspeed);
    motorR->setSpeed(turnspeed);
 }

void goLeft() {
  motorR->run(FORWARD);

  motorR->setSpeed(turnspeed);
  motorL->setSpeed(0);
}

void goRight() {
  motorL->run(FORWARD);

  motorL->setSpeed(turnspeed);
  motorR->setSpeed(0);
}

void goStraight() {
  motorL->run(FORWARD);
  motorR->run(FORWARD);

  motorL->setSpeed(speed);
  motorR->setSpeed(speed);
 }

void decideDirection(int IR_L, int IR_M, int IR_R) {
  if (IR_M >= THRESHOLD) {
    if (IR_L >= THRESHOLD & IR_R < THRESHOLD) {
      goLeftInPlace();
      Serial.println("left in place");
    }
    else if(IR_R >= THRESHOLD & IR_L < THRESHOLD) {
      goRightInPlace();
      Serial.println("right in place");
    }
    else {
      goStraight();
      Serial.println("straight");
    }
  }
  else if (IR_L >=THRESHOLD & IR_R < THRESHOLD) {
    goLeft();
    Serial.println("left");
  }

  else if (IR_R >=THRESHOLD & IR_L < THRESHOLD) {
    goRight();
    Serial.println("right");
  }

  else {
    goStraight();
    Serial.println("straight");
  }
  
}

void setup() {
  Serial.begin(9600);           // set up Serial library at 9600 bps

  if (!AFMS.begin()) {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1);
  }
  Serial.println("Motor Shield found.");

  // Set the speed to start, from 0 (off) to 255 (max speed)
  motorL->setSpeed(150);
  motorR->setSpeed(150);

  motorL->run(FORWARD);
  motorR->run(FORWARD);

  // turn on motor
  motorL->run(RELEASE);
  motorR->run(RELEASE);

  pinMode(IR_L_OUT, OUTPUT);
  pinMode(IR_M_OUT, OUTPUT);
  pinMode(IR_R_OUT, OUTPUT);
}

void loop() {
  int IR_L_value = getReading(IR_L_OUT, IR_L_READ);
  int IR_M_value = getReading(IR_M_OUT, IR_M_READ);
  int IR_R_value = getReading(IR_R_OUT, IR_R_READ);
  Serial.print("Left: ");
  Serial.println(IR_L_value);
  Serial.print("Middle: ");
  Serial.println(IR_M_value);
  Serial.print("Right: ");
  Serial.println(IR_R_value);
  decideDirection(IR_L_value, IR_M_value, IR_R_value);
  delay(50);
}