#include <SoftwareSerial.h>
#include "VoiceRecognitionV3.h"

// --- Pin Definitions ---
// Voice Module (Software Serial)
VR myVR(2, 3);    // RX (to VR TX), TX (to VR RX)

// L298N Motor Driver Pins
#define enA 10    // Enable A (Right Motor Speed)
#define in1 9     // Right Motor Direction
#define in2 8     // Right Motor Direction
#define in3 7     // Left Motor Direction
#define in4 6     // Left Motor Direction
#define enB 5     // Enable B (Left Motor Speed)

// --- Voice Command Constants ---
// Ensure these match the "Signature" IDs you trained your module with!
#define CMD_LEFT     (0)
#define CMD_RIGHT    (1)
#define CMD_FORWARD  (2)
#define CMD_BACKWARD (3)
#define CMD_STOP     (4)

// --- Settings ---
#define MOTOR_SPEED 180 // Set speed (0 - 255)

uint8_t records[7]; // buffer for loading records
uint8_t buf[64];    // buffer for receiving signature

void setup() {
  // 1. Initialize Serial for Debugging
  Serial.begin(115200);
  Serial.println("Initializing Voice Controlled Car...");

  // 2. Initialize Voice Module
  myVR.begin(9600);

  // 3. Initialize Motor Pins
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);
  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);

  // Set initial speed
  analogWrite(enA, MOTOR_SPEED);
  analogWrite(enB, MOTOR_SPEED);
  stopCar(); // Ensure car is stopped at startup

  // 4. Check VR Module Connection
  if (myVR.clear() == 0) {
    Serial.println("VR Module: Recognizer cleared.");
  } else {
    Serial.println("VR Module: Not found! Check connections (Pins 2 & 3).");
    // Don't stop the code here, just warn the user
  }

  // 5. Load Voice Records
  // The module must be trained with these specific index numbers first!
  loadRecord(CMD_LEFT, "Left");
  loadRecord(CMD_RIGHT, "Right");
  loadRecord(CMD_FORWARD, "Forward");
  loadRecord(CMD_BACKWARD, "Backward");
  loadRecord(CMD_STOP, "Stop");
  
  Serial.println("System Ready. Say a command!");
}

void loop() {
  int ret;
  ret = myVR.recognize(buf, 50); // Listen for command

  if (ret > 0) {
    // If a command is recognized, buf[1] holds the Index/Signature
    switch (buf[1]) {
      case CMD_FORWARD:
        Serial.println("Command: FORWARD");
        moveForward();
        break;

      case CMD_BACKWARD:
        Serial.println("Command: BACKWARD");
        moveBackward();
        break;

      case CMD_LEFT:
        Serial.println("Command: LEFT");
        turnLeft();
        break;

      case CMD_RIGHT:
        Serial.println("Command: RIGHT");
        turnRight();
        break;

      case CMD_STOP:
        Serial.println("Command: STOP");
        stopCar();
        break;

      default:
        Serial.println("Unknown Command");
        break;
    }
  }
}

// --- Helper to Load Records safely ---
void loadRecord(uint8_t recordIndex, String name) {
  if (myVR.load(recordIndex) >= 0) {
    Serial.print(name); 
    Serial.println(" loaded.");
  } else {
    Serial.print("Failed to load: "); 
    Serial.println(name);
  }
}

// --- Motor Control Functions ---

void moveForward() {
  // Right Motor Forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Left Motor Forward
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void moveBackward() {
  // Right Motor Backward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // Left Motor Backward
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnLeft() {
  // Right Motor Forward
  digitalWrite(in1, HIGH);
  digitalWrite(in2, LOW);
  // Left Motor Backward (Spin Turn)
  digitalWrite(in3, LOW);
  digitalWrite(in4, HIGH);
}

void turnRight() {
  // Right Motor Backward
  digitalWrite(in1, LOW);
  digitalWrite(in2, HIGH);
  // Left Motor Forward (Spin Turn)
  digitalWrite(in3, HIGH);
  digitalWrite(in4, LOW);
}

void stopCar() {
  digitalWrite(in1, LOW);
  digitalWrite(in2, LOW);
  digitalWrite(in3, LOW);
  digitalWrite(in4, LOW);
}