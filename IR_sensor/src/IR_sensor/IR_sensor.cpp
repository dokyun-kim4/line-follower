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

// Store direction
int direction = 0;

// Define pins
const int IR_L_READ = A2;
const int IR_M_READ = A1;
const int IR_R_READ = A0;

const int IR_L_OUT = 5;
const int IR_M_OUT = 4;
const int IR_R_OUT = 3;

// Other constants
int linspeed = 25;
int turnspeed_default = 30;
const int min_diff = 200;

// Define threshold value for black tape reading
int THRESHOLD = 500; // if over 600, then on black tape

int getReading(int D_pin, int A_pin)
{
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

void goLeftInPlace(int speed)
{
  motorL->run(BACKWARD);
  motorR->run(FORWARD);
  motorL->setSpeed(speed);
  motorR->setSpeed(speed);
}

void goRightInPlace(int speed)
{
  motorR->run(BACKWARD);
  motorL->run(FORWARD);
  motorL->setSpeed(speed);
  motorR->setSpeed(speed);
}

void goStraight()
{
  motorL->run(FORWARD);
  motorR->run(FORWARD);

  motorL->setSpeed(linspeed);
  motorR->setSpeed(linspeed);
}

void decideDirection(int IR_L, int IR_M, int IR_R, int direction)
{
  if ((IR_M > IR_L) & (IR_M > IR_R))
  {
    goStraight();
  }
  else if ((IR_L > IR_R))
  {
    delay(50);
    goLeftInPlace(turnspeed_default);
  }
  else if ((IR_R > IR_L))
  {
    delay(50);
    goRightInPlace(turnspeed_default);
  }
  else
  {
    if (direction == 1)
    {
      goRightInPlace(turnspeed_default);
    }
    else
    {
      goLeftInPlace(turnspeed_default);
    }
    delay(100);
  }
}

void setup()
{
  Serial.begin(9600); // set up Serial library at 9600 bps
  Serial.setTimeout(100);

  if (!AFMS.begin())
  {
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1)
      ;
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

void inputSpeed()
{

  int speedMode = Serial.parseInt();
  switch (speedMode)
  {
  case 0:
    linspeed = 25;
    break;

  case 1:
    linspeed = 30;
    break;

  case 2:
    linspeed = 40;
    break;

  case 3:
    linspeed = 50;
    break;
  }
  Serial.print("Change speed to: ");
  Serial.println(linspeed);
}

void loop()
{
  if (Serial.available() > 0)
  {
    inputSpeed();
  }

  int IR_L_value = getReading(IR_L_OUT, IR_L_READ);
  int IR_M_value = getReading(IR_M_OUT, IR_M_READ);
  int IR_R_value = getReading(IR_R_OUT, IR_R_READ);
  // Serial.print("Left: ");
  // Serial.println(IR_L_value);
  // Serial.print("Middle: ");
  // Serial.println(IR_M_value);
  // Serial.print("Right: ");
  // Serial.println(IR_R_value);
  decideDirection(IR_L_value, IR_M_value, IR_R_value, direction);
  delay(50);
}