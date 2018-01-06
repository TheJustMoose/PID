#include <TimerOne.h>

typedef unsigned long ULONG;

// motor speed sensor pins
// A0 == PF0
const byte LEFT_SENSOR_PIN = PINE4; // Arduino pin 2
#define LEFT_SENSOR_PORT PINE // Do not use "const byte" here, it's not work

volatile ULONG left_cnt = 0;
ULONG last_check = 0;
byte old_val = 0;

long req_speed = 0;

boolean should_update_pid = false;

void on_tmr() {
  should_update_pid = true;
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
  Timer1.initialize(250000); // in us
  Timer1.attachInterrupt(on_tmr);
  Serial.begin(9600);

  attachInterrupt(digitalPinToInterrupt(2), pin_changed, CHANGE);

  pinMode(8, OUTPUT);
  pinMode(13, OUTPUT);

  analogWrite(8, 0);
  digitalWrite(13, LOW);
}

byte _pwm = 255;

void ctrl() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r') {
      req_speed = 50; // 50 max at this freq
      _pwm = 0;
    }
    if (c == 'm') {
      req_speed = 25;
      _pwm = 0;
    }
    if (c == 's') {
      req_speed = 0;
      _pwm = 0;
    }

    if (c == '+' && _pwm < 255)
      _pwm++;
    if (c == '-' && _pwm > 0)
      _pwm--;
    if (c == '*')
      _pwm = 128;
  }
}

struct SPid {
  long dState;                  // Last position input
  long iState;                  // Integrator state
  long iMax, iMin;

  // 1/65536 == 0,000015
  //  1/8192 == 0,00012
  //    1/64 == 0,015625

  // Maximum and minimum allowable integrator state
  long iGain,        // integral gain (0.0001 .. 0.01) ( * 1-64  / 8192)
       pGain,        // proportional gain (1 .. 100)
       dGain;        // derivative gain (<= 1.0) ( * 1-256 / 256)

  SPid():
    iGain(long(0.0*65536)), pGain(long(3.3*65536)), dGain(long(0.0*65536)) { // 05
    dState = 0;
    iState = 0;
    iMin = 0;
    iMax = 255;
  }

  long UpdatePID(long error, long position);
};


long SPid::UpdatePID(long error, long position) {
  iState += error;          // calculate the integral state with appropriate limiting
  if (iState > iMax)
      iState = iMax;
  else if (iState < iMin)
      iState = iMin;

  long res = pGain * error +  // calculate the proportional term
             iGain * iState - // calculate the integral term
             dGain * (position - dState); // сумматор 2
  res /= 65536;

  dState = position;
  return res;
}

SPid pid;

void loop() {
  ctrl();
  if (!should_update_pid)
    return;

  should_update_pid = false;

  if (_pwm) {
    analogWrite(8, _pwm);
    Serial.println(_pwm);
    return;
  }

  if (req_speed == 0) {
    analogWrite(8, 0);
    return;
  }

  ULONG s = left(); // real speed
  clr_left();

  long act = pid.UpdatePID(
               req_speed - s, // sum 1
               s); // AVR PWM should work in range 0 - 255

  Serial.print(act);
  if (act < 0)
    act = 100; // min pwm val for motor
  if (act > 255)
    act = 255;

  analogWrite(8, 255);

  Serial.print(" ");
  Serial.println(s);
}

