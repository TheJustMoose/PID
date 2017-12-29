#include <TimerOne.h>

typedef unsigned long ULONG;

// motor speed sensor pins
// A0 == PF0
const byte LEFT_SENSOR_PIN = PINF0;  // Arduino pin 5

volatile ULONG left_cnt = 0;
boolean left_encoder_val = false;

void on_tmr() {
  byte val = PINF;
  boolean vbl = val & _BV(LEFT_SENSOR_PIN);
  if (vbl != left_encoder_val) {
    left_encoder_val = vbl;
    left_cnt++;
  }
}

ULONG left() {
  noInterrupts();
  ULONG res = left_cnt;
  interrupts();
  return res;
}

void clr_left() {
  noInterrupts();
  left_cnt = 0;
  interrupts();
}

void setup() {
  Timer1.initialize(10000); // in us
  Timer1.attachInterrupt(on_tmr);
  Serial.begin(9600);

  pinMode(8, OUTPUT);
  digitalWrite(8, LOW);
}

void ctrl() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r')
      digitalWrite(8, HIGH);
    if (c == 's')
      digitalWrite(8, LOW);
  }
}

void loop() {
  ctrl();
  delay(1000); // 1s
  ULONG val = left();
  clr_left();
  Serial.println(val);
}

