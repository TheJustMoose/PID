#include <TimerOne.h>

typedef unsigned long ULONG;

// motor speed sensor pins
// A0 == PF0
const byte LEFT_SENSOR_PIN = PINE4; // Arduino pin 2
#define LEFT_SENSOR_PORT PINE // Do not use "const byte" here, it's not work

volatile ULONG left_cnt = 0;
ULONG last_check = 0;
byte old_val = 0;

void on_tmr() {
}

void pin_changed() {
  // should not check pin too often
  // I think it fail for good signal :(
  if (last_check == millis())
    return;
  last_check = millis();

  byte val = LEFT_SENSOR_PORT & _BV(LEFT_SENSOR_PIN);
  if (val == old_val)
    return;

  left_cnt++;
  old_val = val;

  PORTB = val ? 255 : 0;
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
  Timer1.initialize(1000); // in us
  Timer1.attachInterrupt(on_tmr);
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(2), pin_changed, CHANGE);

  pinMode(8, OUTPUT);
  pinMode(13, OUTPUT);

  analogWrite(8, 0);
  digitalWrite(13, LOW);
}

void ctrl() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r')
      analogWrite(8, 255);
    if (c == 'm')
      analogWrite(8, 192);
    if (c == 's')
      analogWrite(8, 0);
  }
}

void loop() {
  ctrl();
  delay(1000); // 1s
  ULONG val = left();
  clr_left();
  Serial.println(val);
}

