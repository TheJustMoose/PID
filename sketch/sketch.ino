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
    Init();
  }

  void Init() {
    dState = 0;
    iState = 160; // magic value for smooth start
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

void Init() {
  LeftPid.Init();
  RightPid.Init();
  LeftSensor.Clear();
  RightSensor.Clear();
}

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
    should_update_pid = true; // every 500ms
    call_cnt = 0;
  }
}

void go(bool fwd) {
  digitalWrite(51, !fwd);
  digitalWrite(53, fwd);
  digitalWrite(47, fwd);
  digitalWrite(49, !fwd);
}

void setup() {
  Timer1.initialize(500); // in us
  Timer1.attachInterrupt(on_tmr);
  //Serial.begin(115200);
  Serial3.begin(38400); // to HC-06
  Serial3.println("Bluetooth started!");

  // left pins
  pinMode(left_pwm_pin, OUTPUT);
  pinMode(51, OUTPUT);
  pinMode(53, OUTPUT);
  // left forward
  analogWrite(left_pwm_pin, 0);

  // right pins
  pinMode(right_pwm_pin, OUTPUT);
  pinMode(47, OUTPUT);
  pinMode(49, OUTPUT);
  // right forward
  analogWrite(right_pwm_pin, 0);

  Init();

  Serial3.print(255); // begin marker
  Serial3.print(" ");
  Serial3.print(255);
  Serial3.print("\n");
}

byte _pwm = 0;
int loop_cnt = 0;

void ctrl() {
  if (Serial3.available() > 0) {
    char c = Serial3.read();
    if (c == 'r') { // run
      req_speed = 120; // max at this freq
      _pwm = 0;
      loop_cnt = 0;
      go(true); // forward
      Serial3.println("Ok, run");
    }
    else if (c == 'm') { // middle == run slow
      req_speed = 80;
      _pwm = 0;
      Serial3.println("Ok, slow run");
    }
    else if (c == 's') { // stop
      req_speed = 0;
      _pwm = 0;
      Serial3.println("Ok, stop");
    }
    else if (c == 'd') { // debug
      Serial3.print(LeftPid.iState);
      Serial3.print(" ");
      Serial3.print(RightPid.iState);
      Serial3.print("\n");
    }
    else { // loopback
      Serial3.print("Ok, cmd: ");
      Serial3.println(c);
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

  // every 500ms
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
    Serial3.print(_pwm);
    Serial3.print(" ");
    Serial3.print(ls);
    Serial3.print(" ");
    Serial3.println(rs);
    return;
  }  

  loop_cnt++;

  if (loop_cnt == 25) { // more than 3s
    go(false); // backward
    analogWrite(left_pwm_pin, 0);
    analogWrite(right_pwm_pin, 0);
    Init();
    return;
  }
  else if (loop_cnt > 50) { // more than 6s
    go(true); // forward
    req_speed = 0; // stop!
    return;
  }

  const int thres = 4;
  if (loop_cnt < thres ||
     (loop_cnt > 25 && loop_cnt < 25 + thres)) {
    analogWrite(left_pwm_pin, 160);
    analogWrite(right_pwm_pin, 160);
    Serial3.print("Diff lock - ");
  }
  else { // diff lock ;)
    long lact = LeftPid.UpdatePID(
               req_speed - ls, // sum 1
               ls); // AVR PWM should work in range 0 - 255
    long ract = RightPid.UpdatePID(
               req_speed - rs, // sum 1
               rs); // AVR PWM should work in range 0 - 255

    Serial3.print(lact);
    LimitValue(lact);

    Serial3.print(" ");
    Serial3.print(ract);
    LimitValue(ract);

    analogWrite(left_pwm_pin, lact);
    analogWrite(right_pwm_pin, ract);
  }

  Serial3.print(" s: ");
  Serial3.print(ls);
  Serial3.print(" ");
  Serial3.println(rs);
}

