#include <TimerOne.h>

typedef unsigned long ULONG;

// motor speed sensor pins
const byte LEFT_SENSOR_PIN = PINE4; // Arduino pin 2
const byte RIGHT_SENSOR_PIN = PINE5; // Arduino pin 3
#define SENSOR_PORT PINE // Do not use "const byte" here, it's not work

const int left_pwm_pin = 46;
const int right_pwm_pin = 44;

class OpticalSensor {
 private:
  volatile ULONG cnt_ = 0;
  volatile ULONG white_cnt_ = 0;
  int pin_;

 public:
  OpticalSensor(int pin);

  ULONG Get();
  void Clear();
  void Check();
};

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
    iGain(long(1.0*65536)), pGain(long(1.0*65536)), dGain(long(0.0*65536)) {
    //iGain(long(0.95*65536)), pGain(long(2.7*65536)), dGain(long(0.002*65536)) {
    dState = 0;
    iState = 0;
    iMin = 0;
    iMax = 255;
  }

  long UpdatePID(long error, long position);
};


long req_speed = 0;
uint16_t call_cnt = 0;

boolean should_update_pid = false;

OpticalSensor LeftSensor(LEFT_SENSOR_PIN);
OpticalSensor RightSensor(RIGHT_SENSOR_PIN);

SPid LeftPid;
SPid RightPid;


OpticalSensor::OpticalSensor(int pin)
  : pin_(pin) {
}

void OpticalSensor::Check() {
  if (SENSOR_PORT & _BV(pin_)) { // white zone on encoder
    white_cnt_++;
    if (white_cnt_ == 3) // just one time for zone
      cnt_++;
  }
  else
    white_cnt_ = 0;
}

ULONG OpticalSensor::Get() {
  noInterrupts();
  ULONG res = cnt_;
  interrupts();
  return res;
}

void OpticalSensor::Clear() {
  noInterrupts();
  cnt_ = 0;
  interrupts();
}

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

void on_tmr() { // called every 500us
  LeftSensor.Check();
  RightSensor.Check();

  call_cnt++;
  if (call_cnt == 1000) {
    should_update_pid = true;
    call_cnt = 0;
  }
}

void setup() {
  Timer1.initialize(500); // in us
  Timer1.attachInterrupt(on_tmr);
  Serial.begin(115200);
  Serial.println("Started!");

  // left pins
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(51, OUTPUT);
  pinMode(53, OUTPUT);
  // left forward
  analogWrite(left_pwm_pin, 0);
  digitalWrite(51, HIGH);
  digitalWrite(53, LOW);

  // right pins
  pinMode(right_pwm_pin, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(49, OUTPUT);
  // right forward
  analogWrite(right_pwm_pin, 0);
  digitalWrite(47, LOW);
  digitalWrite(49, HIGH);
}

byte _pwm = 0;

void ctrl() {
  if (Serial.available() > 0) {
    char c = Serial.read();
    if (c == 'r') {
      req_speed = 120; // max at this freq
      _pwm = 0;
    }
    if (c == 'm') {
      req_speed = 80;
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
    if (c == '\\')
      _pwm = 225;
    if (c == '#')
      _pwm = 255;
  }
}

void LimitValue(long &act) {
  if (act < 0)
    act = 0; // min pwm val for motor
  if (act > 250)
    act = 250;
}

void loop() {
  ctrl();
  if (!should_update_pid)
    return;

  should_update_pid = false;

  if (req_speed == 0 && !_pwm) {
    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);
    return;
  }

  ULONG ls = LeftSensor.Get(); // real speed
  ULONG rs = RightSensor.Get();
  LeftSensor.Clear();
  RightSensor.Clear();

  if (_pwm) {
    analogWrite(left_pwm_pin, _pwm);
    analogWrite(right_pwm_pin, _pwm);
    Serial.print(_pwm);
    Serial.print(" ");
    Serial.print(ls);
    Serial.print(" ");
    Serial.println(rs);
    return;
  }

  long lact = LeftPid.UpdatePID(
               req_speed - ls, // sum 1
               ls); // AVR PWM should work in range 0 - 255
  long ract = RightPid.UpdatePID(
               req_speed - rs, // sum 1
               rs); // AVR PWM should work in range 0 - 255

  Serial.print(lact);
  LimitValue(lact);

  Serial.print(" ");
  Serial.print(ract);
  LimitValue(ract);

  analogWrite(left_pwm_pin, lact);
  analogWrite(right_pwm_pin, ract);

  Serial.print(" s: ");
  Serial.print(ls);
  Serial.print(" ");
  Serial.println(rs);
}

