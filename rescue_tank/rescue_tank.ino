/*
  FallGuard rescue tank
  Keyestudio Mini Tank Robot V2 (KS0428): V4.0 (UNO) board + L298P motor shield,
  HM-10 Bluetooth module ("HMSoft") on the hardware serial port at 9600 baud.

  Commands (one character each, sent by robot_link.py over Bluetooth):
    F forward   B back   L spin left   R spin right   S stop
    Steering while driving forward (one track slower than the other):
      a = curve hard left   s = veer slightly left   d = veer slightly right   g = curve hard right
    X rescue routine (drive out, flash the medic cross)
    1-4 motor test (keys 1-4 in the sketch): spins ONE track for 1 second
      1 = left track,  direction pin LOW      2 = left track,  direction pin HIGH
      3 = right track, direction pin LOW      4 = right track, direction pin HIGH
      (On this tank: LOW = backward, HIGH = forward, hence the FLIP settings.)

  Safety: while driving, the tank stops by itself if no command arrives for
  0.7 s, so it never runs away if the Mac or Bluetooth drops.

  UPLOAD TIP: unplug the Bluetooth module before uploading, then plug it back in.
  (Keyestudio's docs say the upload fails otherwise.)
*/

// ---- pins (Keyestudio's KS0428 pins, left/right swapped to match this tank) ----
#define ML_Ctrl 12   // left motor direction
#define ML_PWM   3   // left motor speed
#define MR_Ctrl 13   // right motor direction
#define MR_PWM  11   // right motor speed
#define SCL_Pin A5   // 8x16 LED matrix clock
#define SDA_Pin A4   // 8x16 LED matrix data

// ---- motor setup: set these to match YOUR tank after running the 1-4 test ----
const bool LEFT_FLIP   = true;   // true if the left track runs backward when it should go forward
const bool RIGHT_FLIP  = true;   // true if the right track runs backward when it should go forward
const bool PWM_INVERTS = false;  // true if tests 2 and 4 did nothing (this board wants 255 - speed on HIGH)

// ---- tuning ----
const int DRIVE_SPEED = 255;          // 0-255 (full power: tracks need it on carpet)
const int TURN_SPEED  = 255;
const int VEER_SLOW = 170;            // slower track speed for a slight veer
const int CURVE_SLOW = 70;            // slower track speed for a hard curve
const unsigned long WATCHDOG_MS = 700;
const unsigned long RESCUE_DRIVE_MS = 1500;  // how long X drives forward; change for your room

// ---- LED matrix pictures (16 columns, 8 rows each) ----
unsigned char face_ok[16] = {0x00,0x00,0x1c,0x22,0x41,0x55,0x41,0x22,0x22,0x41,0x55,0x41,0x22,0x1c,0x00,0x00};
unsigned char cross[16]   = {0x00,0x00,0x00,0x18,0x18,0x18,0xff,0xff,0xff,0xff,0x18,0x18,0x18,0x00,0x00,0x00};
unsigned char arrow_up[16]= {0x00,0x00,0x00,0x00,0x08,0x0c,0xfe,0xff,0xfe,0x0c,0x08,0x00,0x00,0x00,0x00,0x00};
unsigned char blank[16]   = {0};

void carForward(int s); void carBack(int s); void carLeft(int s); void carRight(int s); void carStop();
void motor(int ctrlPin, int pwmPin, bool flip, int speed); void motorTest(int ctrlPin, int pwmPin, int dir);
void carCurve(int leftSpeed, int rightSpeed);
void rescue(); void matrixShow(unsigned char pic[]);
void iicStart(); void iicSend(unsigned char data); void iicEnd();

bool moving = false;
unsigned long lastCmdMs = 0;

void setup() {
  Serial.begin(9600);
  pinMode(ML_Ctrl, OUTPUT);
  pinMode(ML_PWM, OUTPUT);
  pinMode(MR_Ctrl, OUTPUT);
  pinMode(MR_PWM, OUTPUT);
  pinMode(SCL_Pin, OUTPUT);
  pinMode(SDA_Pin, OUTPUT);
  carStop();
  matrixShow(face_ok);
}

void loop() {
  if (Serial.available()) {
    char c = Serial.read();
    lastCmdMs = millis();
    switch (c) {
      case 'F': carForward(DRIVE_SPEED); matrixShow(arrow_up); break;
      case 'B': carBack(DRIVE_SPEED);    break;
      case 'L': carLeft(TURN_SPEED);     break;
      case 'R': carRight(TURN_SPEED);    break;
      case 'S': carStop(); matrixShow(face_ok); break;
      case 'a': carCurve(CURVE_SLOW, DRIVE_SPEED); break;
      case 's': carCurve(VEER_SLOW,  DRIVE_SPEED); break;
      case 'd': carCurve(DRIVE_SPEED, VEER_SLOW);  break;
      case 'g': carCurve(DRIVE_SPEED, CURVE_SLOW); break;
      case 'X': rescue(); break;
      case '1': motorTest(ML_Ctrl, ML_PWM, LOW);  break;
      case '2': motorTest(ML_Ctrl, ML_PWM, HIGH); break;
      case '3': motorTest(MR_Ctrl, MR_PWM, LOW);  break;
      case '4': motorTest(MR_Ctrl, MR_PWM, HIGH); break;
      default: break;   // ignore newlines and anything else
    }
  }
  if (moving && millis() - lastCmdMs > WATCHDOG_MS) {
    carStop();
    matrixShow(face_ok);
  }
}

// ---- rescue routine (step 1: fixed route; step 2 will steer by camera) ----
void rescue() {
  matrixShow(cross);
  carForward(DRIVE_SPEED);
  delay(RESCUE_DRIVE_MS);
  carStop();
  for (int i = 0; i < 6; i++) {          // flash the cross: "help is here"
    matrixShow(blank);
    delay(250);
    matrixShow(cross);
    delay(250);
  }
  while (Serial.available()) Serial.read();   // drop commands that piled up during the rescue
  lastCmdMs = millis();
}

// ---- motors ----
// speed: -255 (full backward) .. +255 (full forward), after the FLIP settings
void motor(int ctrlPin, int pwmPin, bool flip, int speed) {
  if (flip) speed = -speed;
  speed = constrain(speed, -255, 255);
  if (speed >= 0) {                       // direction pin LOW
    digitalWrite(ctrlPin, LOW);
    analogWrite(pwmPin, speed);
  } else {                                // direction pin HIGH
    digitalWrite(ctrlPin, HIGH);
    analogWrite(pwmPin, PWM_INVERTS ? 255 + speed : -speed);   // 255 - |speed| on inverting boards
  }
}

void carForward(int s) { motor(ML_Ctrl, ML_PWM, LEFT_FLIP,  s); motor(MR_Ctrl, MR_PWM, RIGHT_FLIP,  s); moving = true; }
void carBack(int s)    { motor(ML_Ctrl, ML_PWM, LEFT_FLIP, -s); motor(MR_Ctrl, MR_PWM, RIGHT_FLIP, -s); moving = true; }
void carLeft(int s)    { motor(ML_Ctrl, ML_PWM, LEFT_FLIP, -s); motor(MR_Ctrl, MR_PWM, RIGHT_FLIP,  s); moving = true; }
void carRight(int s)   { motor(ML_Ctrl, ML_PWM, LEFT_FLIP,  s); motor(MR_Ctrl, MR_PWM, RIGHT_FLIP, -s); moving = true; }
void carCurve(int l, int r) { motor(ML_Ctrl, ML_PWM, LEFT_FLIP, l); motor(MR_Ctrl, MR_PWM, RIGHT_FLIP, r); moving = true; }
void carStop()         { motor(ML_Ctrl, ML_PWM, false, 0);    motor(MR_Ctrl, MR_PWM, false, 0);    moving = false; }

// raw test: one track, given direction pin level, PWM 255, for 1 second
void motorTest(int ctrlPin, int pwmPin, int dir) {
  carStop();
  digitalWrite(ctrlPin, dir);
  analogWrite(pwmPin, 255);
  delay(1000);
  carStop();
  while (Serial.available()) Serial.read();
  lastCmdMs = millis();
}

// ---- 8x16 LED matrix (bit-banged, same protocol as Keyestudio's examples) ----
void matrixShow(unsigned char pic[]) {
  iicStart();
  iicSend(0xC0);                 // start at the first column
  for (int i = 0; i < 16; i++) iicSend(pic[i]);
  iicEnd();
  iicStart();
  iicSend(0x8A);                 // display on, medium brightness
  iicEnd();
}

void iicStart() {
  digitalWrite(SCL_Pin, HIGH); delayMicroseconds(3);
  digitalWrite(SDA_Pin, HIGH); delayMicroseconds(3);
  digitalWrite(SDA_Pin, LOW);  delayMicroseconds(3);
}

void iicSend(unsigned char data) {
  for (int i = 0; i < 8; i++) {
    digitalWrite(SCL_Pin, LOW); delayMicroseconds(3);
    digitalWrite(SDA_Pin, (data & 0x01) ? HIGH : LOW); delayMicroseconds(3);
    digitalWrite(SCL_Pin, HIGH); delayMicroseconds(3);
    data >>= 1;
  }
}

void iicEnd() {
  digitalWrite(SCL_Pin, LOW);  delayMicroseconds(3);
  digitalWrite(SDA_Pin, LOW);  delayMicroseconds(3);
  digitalWrite(SCL_Pin, HIGH); delayMicroseconds(3);
  digitalWrite(SDA_Pin, HIGH); delayMicroseconds(3);
}
