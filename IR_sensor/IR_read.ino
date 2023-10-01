// Define pins
const int IR_PIN = 6;
const int V_OUT = A5;

void setup() {
  Serial.begin(9600);
  pinMode(IR_PIN, OUTPUT);
}

void loop() {
  // IR LED on
  digitalWrite(IR_PIN, HIGH);
  delay(1000);
  // Read phototransistor voltage
  int v_out = analogRead(V_OUT);
  Serial.println(v_out);
}
