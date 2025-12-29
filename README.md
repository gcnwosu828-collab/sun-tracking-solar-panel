# sun-tracking-solar-panel
Group 7
/*
  SunTracker.ino
  Two-axis sun tracker using 4 LDRs (top-left, top-right, bottom-left, bottom-right)
  and 2 hobby servos (pan = azimuth, tilt = elevation).

  Wiring:
  - TL (Top Left)  -> A0 (voltage divider: 5V - LDR - A0 - 10k - GND)
  - TR (Top Right) -> A1
  - BL (Bot Left)  -> A2
  - BR (Bot Right) -> A3
  - Pan servo signal -> D9
  - Tilt servo signal -> D10

  Notes:
  - Use separate servo power if needed (common ground).
  - Tune Kp, deadband, and maxStep to your system.
*/

#include <Servo.h>

// Sensor pins
const uint8_t PIN_TL = A0;
const uint8_t PIN_TR = A1;
const uint8_t PIN_BL = A2;
const uint8_t PIN_BR = A3;

// Servo pins
const uint8_t PIN_SERVO_PAN  = 9;  // azimuth
const uint8_t PIN_SERVO_TILT = 10; // elevation

Servo servoPan;
Servo servoTilt;

// Control params (tune these)
float Kp = 0.02;           // proportional gain (maps ADC error -> degrees)
int deadband = 30;        // ADC units below which we don't move
int sampleCount = 8;      // average readings to reduce noise
int loopDelayMs = 200;    // delay between control updates
int maxStep = 3;          // maximum degrees to move per loop (limits jerk)

// Servo mechanical limits (adjust to your mount)
int panMin = 0;
int panMax = 180;
int tiltMin = 10;   // avoid pointing below horizon if not needed
int tiltMax = 170;

// Starting (center) positions
int panPos = 90;
int tiltPos = 90;

void setup() {
  Serial.begin(115200);
  servoPan.attach(PIN_SERVO_PAN);
  servoTilt.attach(PIN_SERVO_TILT);

  // initialize servos at center
  servoPan.write(panPos);
  servoTilt.write(tiltPos);
  delay(500);
  Serial.println("SunTracker starting...");
  Serial.println("Use a flashlight to test responsiveness.");
}

int readAverage(uint8_t pin, int count) {
  long sum = 0;
  for (int i = 0; i < count; ++i) {
    sum += analogRead(pin);
    delay(2); // small delay between samples
  }
  return (int)(sum / count);
}

void loop() {
  // Read sensors (averaged)
  int tl = readAverage(PIN_TL, sampleCount);
  int tr = readAverage(PIN_TR, sampleCount);
  int bl = readAverage(PIN_BL, sampleCount);
  int br = readAverage(PIN_BR, sampleCount);

  // Compute aggregated signals
  int leftSum  = tl + bl;
  int rightSum = tr + br;
  int topSum   = tl + tr;
  int botSum   = bl + br;

  // Errors: positive -> move toward left/top
  int errH = leftSum - rightSum; // horizontal (azimuth)
  int errV = topSum - botSum;    // vertical (elevation)

  // Debug prints
  Serial.print("TL:"); Serial.print(tl);
  Serial.print(" TR:"); Serial.print(tr);
  Serial.print(" BL:"); Serial.print(bl);
  Serial.print(" BR:"); Serial.print(br);
  Serial.print(" | errH:"); Serial.print(errH);
  Serial.print(" errV:"); Serial.print(errV);

  // Decide horizontal movement
  if (abs(errH) > deadband) {
    float deltaDegH = Kp * errH; // can be negative
    // limit maximum step magnitude
    if (deltaDegH > maxStep) deltaDegH = maxStep;
    if (deltaDegH < -maxStep) deltaDegH = -maxStep;
    panPos += (int)round(deltaDegH);
  }

  // Decide vertical movement
  if (abs(errV) > deadband) {
    float deltaDegV = Kp * errV; // positive -> move tilt upward
    if (deltaDegV > maxStep) deltaDegV = maxStep;
    if (deltaDegV < -maxStep) deltaDegV = -maxStep;
    // invert sign if your servo orientation is reversed for tilt
    tiltPos += (int)round(deltaDegV);
  }

  // Constrain to servo limits
  panPos = constrain(panPos, panMin, panMax);
  tiltPos = constrain(tiltPos, tiltMin, tiltMax);

  // Move servos
  servoPan.write(panPos);
  servoTilt.write(tiltPos);

  Serial.print(" | pan:"); Serial.print(panPos);
  Serial.print(" tilt:"); Serial.println(tiltPos);

  delay(loopDelayMs);
}
