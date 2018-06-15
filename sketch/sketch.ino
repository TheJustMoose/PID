#include <TimerOne.h>

typedef unsigned long ULONG;

// motor speed sensor pins
const byte LEFT_SENSOR_PIN = PINE4; // Arduino pin 2
#define LEFT_SENSOR_PORT PINE // Do not use "const byte" here, it's not work

class OpticalSensor {
 public:
  volatile ULONG cnt_ = 0;

  ULONG left();
};

OpticalSensor Left;

ULONG white_cnt = 0;

long req_speed = 0;
uint16_t call_cnt = 0;

boolean should_update_pid = false;

void on_tmr() { // called every 500us
  byte val = LEFT_SENSOR_PORT & _BV(LEFT_SENSOR_PIN);
  if (!(val & 0x10)) { // white zone on encoder
    white_cnt++;
    if (white_cnt == 5) // just one time for zone
      Left.cnt_++;
  }
  else
    white_cnt = 0;

  call_cnt++;
  if (call_cnt == 1000) {
    should_update_pid = true;
    call_cnt = 0;
  }
}

ULONG OpticalSensor::left() {
  noInterrupts();
  ULONG res = Left.cnt_;
  interrupts();
  return res;
}

void clr_left() {
  noInterrupts();
  Left.cnt_ = 0;
  interrupts();
}

void setup() {
  Timer1.initialize(500); // in us
  Timer1.attachInterrupt(on_tmr);
  Serial.begin(115200);

  pinMode(8, OUTPUT);
  analogWrite(8, 0);
}

byte _pwm = 0;

void ctrl() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r') {
      req_speed = 100; // max at this freq
      _pwm = 0;
    }
    if (c == 'm') {
      req_speed = 50;
      _pwm = 0;
    }
    if (c == 's') {
      req_speed = 0;
      _pwm = 0;
    }

    if (c == '+' && _pwm < 255)
      _pwm ++;
    if (c == '-' && _pwm > 0)
      _pwm --;
    if (c == '*')
      _pwm = 128;
    if (c == '/')
      _pwm = 160;
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
    iGain(long(0.95*65536)), pGain(long(2.7*65536)), dGain(long(0.002*65536)) { // 05
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

  if (req_speed == 0 && !_pwm) {
    analogWrite(8, 0);
    return;
  }

  ULONG s = Left.left(); // real speed
  clr_left();

  if (_pwm) {
    analogWrite(8, _pwm);
    Serial.print(_pwm);
    Serial.print(" ");
    Serial.println(s);
    return;
  }

  long act = pid.UpdatePID(
               req_speed - s, // sum 1
               s); // AVR PWM should work in range 0 - 255

  Serial.print(act);
  if (act < 0)
    act = 0; // min pwm val for motor
  if (act > 255)
    act = 255;

  analogWrite(8, act);

  Serial.print(" ");
  Serial.print(act);
  Serial.print(" ");
  Serial.println(s);
}

