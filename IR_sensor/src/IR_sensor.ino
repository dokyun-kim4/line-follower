#include "Arduino.h"

// Define pins
const int IR1_READ = A6;
const int IR2_READ = A5;
const int IR3_READ = A4;

const int IR1_OUT = 6;
const int IR2_OUT = 5;
const int IR3_OUT = 4;


void setup() {
  Serial.begin(9600);
  pinMode(IR1_OUT, OUTPUT);
  pinMode(IR2_OUT, OUTPUT);
  pinMode(IR3_OUT, OUTPUT);
}

void loop() {
  // IR LED on
  digitalWrite(IR1_OUT, HIGH);
  digitalWrite(IR2_OUT, HIGH);
  digitalWrite(IR3_OUT, HIGH);
  delay(100);

  // Read phototransistor voltage
  int v_on_1 = analogRead(IR1_READ);
  int v_on_2 = analogRead(IR2_READ);
  int v_on_3 = analogRead(IR3_READ);

  // IR LED off
  digitalWrite(IR1_OUT, LOW);
  digitalWrite(IR2_OUT, LOW);
  digitalWrite(IR3_OUT, LOW);
  delay(100);

  // Read phototransistor voltage
  int v_off_1 = analogRead(IR1_READ);
  int v_off_2 = analogRead(IR2_READ);
  int v_off_3 = analogRead(IR3_READ);

  // Subtract the ambient noise
  Serial.println(v_on_1 - v_off_1);
  Serial.println(v_on_2 - v_off_2);
  Serial.println(v_on_3 - v_off_3);
}