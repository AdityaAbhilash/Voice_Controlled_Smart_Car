#include <SoftwareSerial.h>
#include <Servo.h>

// Bluetooth (RX,TX)
SoftwareSerial bluetooth(11, 12); // RX, TX

char t = 0;

// L298N pins
#define enA 10   // Left motor enable (we will drive this HIGH to avoid Timer1 conflict)
#define in1 9
#define in2 8

#define enB 5    // Right motor PWM (still safe to use analogWrite)
#define in3 7
#define in4 6

// Ultrasonic pins (analog pins used as digital)
const uint8_t TRIG_PIN = A3; // trigger
const uint8_t ECHO_PIN = A2; // echo

// Servo
Servo headServo;
const uint8_t SERVO_PIN = A5;

// Servo positions (degrees)
const int SERVO_LEFT = 0;
const int SERVO_FRONT = 90;
const int SERVO_RIGHT = 180;

// Servo state machine timing (ms)
const unsigned long SERVO_STAGE_TIME = 120; // small for "very fast" sweep between stages
// We'll step between left->front->right quickly using small increments

// Servo movement variables
unsigned long lastServoMove = 0;
int servoTarget = SERVO_FRONT;
int servoPos = SERVO_FRONT;
int servoStep = 10; // step degrees per servo update; larger -> faster
int servoStage = 0; // 0=left->front,1=front->right,2=right->front, cycle...

// Ultrasonic timing
const unsigned long ULTRA_INTERVAL = 120; // sample interval (ms)
unsigned long lastUltra = 0;
long distanceCM = 999;

// Safety threshold (cm)
const long THRESHOLD_CM = 20; // stop if object closer than this

// Option: allow motors only when not "safe stop"
bool safeStopped = false;

void setup() {
  Serial.begin(9600);
  bluetooth.begin(9600);

  // Motor pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Ultrasonic pins
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  // Servo attach (uses Timer1)
  headServo.attach(SERVO_PIN);

  // Initialize motors: enable both
  digitalWrite(enA, HIGH);   // ENA HIGH (full enable). Avoid analogWrite on pin 10 to keep Timer1 for Servo.
  analogWrite(enB, 200);     // enB still PWM-capable (pin 5)

  stopMotor();

  // Servo start at front
  servoPos = SERVO_FRONT;
  headServo.write(servoPos);
  lastServoMove = millis();

  Serial.println("Setup complete");
}

// ---------- Motor control helpers ----------
void forwardMotors() {
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void backwardMotors() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void leftMotors() { // pivot left: left motor reverse, right forward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);

  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void rightMotors() { // pivot right: left forward, right reverse
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);

  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void stopMotor() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}

// ---------- Ultrasonic function (returns cm) ----------
long readUltrasonicCM() {
  // Trigger a 10us pulse
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  // Read echo (timeout 30ms -> 30,000us)
  unsigned long duration = pulseIn(ECHO_PIN, HIGH, 30000UL); // in microseconds
  if (duration == 0) {
    // no echo received (out of range)
    return 999;
  }
  long cm = duration / 29UL / 2UL;
  return cm;
}

// ---------- Servo non-blocking fast 3-stage sweep ----------
void updateServoNonBlocking() {
  unsigned long now = millis();
  if (now - lastServoMove < SERVO_STAGE_TIME) return;
  lastServoMove = now;

  // Decide next target based on stage
  // We want three positions: LEFT -> FRONT -> RIGHT -> FRONT -> LEFT ...
  // We'll move in increments of servoStep degrees
  int target;
  if (servoStage == 0) target = SERVO_LEFT;
  else if (servoStage == 1) target = SERVO_FRONT;
  else if (servoStage == 2) target = SERVO_RIGHT;
  else target = SERVO_FRONT;

  // Move servoPos towards target by servoStep
  if (servoPos < target) {
    servoPos += servoStep;
    if (servoPos > target) servoPos = target;
    headServo.write(servoPos);
  } else if (servoPos > target) {
    servoPos -= servoStep;
    if (servoPos < target) servoPos = target;
    headServo.write(servoPos);
  } else {
    // reached target -> advance stage
    servoStage++;
    if (servoStage > 2) servoStage = 0;
  }
}

// ---------- Main loop ----------
void loop() {
  unsigned long now = millis();

  // 1) Bluetooth read (non-blocking)
  if (bluetooth.available() > 0) {
    t = bluetooth.read();
    Serial.print("BT: ");
    Serial.println(t);
  }

  // 2) Ultrasonic sampling periodically (non-blocking by interval)
  if (now - lastUltra >= ULTRA_INTERVAL) {
    lastUltra = now;
    distanceCM = readUltrasonicCM();
    Serial.print("Distance (cm): ");
    Serial.println(distanceCM);

    // Safety check
    if (distanceCM <= THRESHOLD_CM) {
      // Stop immediately
      safeStopped = true;
      stopMotor();
      Serial.println("SAFE STOP: Obstacle too close!");
    } else {
      // If safe and previously stopped by obstacle, allow motion again
      if (safeStopped) {
        safeStopped = false;
        Serial.println("SAFE CLEAR: Motion allowed");
      }
    }
  }

  // 3) Update servo continuously (fast three stage)
  updateServoNonBlocking();

  // 4) Motor control according to Bluetooth command, but only if not safeStopped
  if (!safeStopped) {
    switch (t) {
      case 'F': // forward
        forwardMotors();
        break;

      case 'X': // backward (user used X earlier)
        backwardMotors();
        break;

      case 'L': // left
        leftMotors();
        break;

      case 'R': // right
        rightMotors();
        break;

      case 'S': // stop
        stopMotor();
        break;

      default:
        // If no command, do nothing (or keep last command)
        break;
    }
  } else {
    // remain stopped while safeStopped=true
    stopMotor();
  }

  // small yield to let interrupts and softserial operate
  delay(5);
}