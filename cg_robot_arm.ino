#include <avr/io.h>
//#include "Arduino.h"

// Base 0: right
// Base 180: left
// Shoulder 0: back
// Shoulder 180: front
// Elbow 0: down
// Elbow 180: up
// Gripper 0: open
// Gripper 180: close

const int BASE_MASK     = (1 << PC0); // A0
const int SHOULDER_MASK = (1 << PC1); // A1
const int ELBOW_MASK    = (1 << PC2); // A2
const int GRIPPER_MASK  = (1 << PC3); // A3

// MAX servo pulse is 5000
// MIN servo pulse is 1000

// CALIBRATE
const int BASE_MIN = 1000, BASE_MAX = 5000, BASE_RANGE = BASE_MAX - BASE_MIN;
const int SHOULDER_MIN = 1000, SHOULDER_MAX = 2778, SHOULDER_RANGE = SHOULDER_MAX - SHOULDER_MIN;
const int ELBOW_MIN = 1000, ELBOW_MAX = 2778, ELBOW_RANGE = ELBOW_MAX - ELBOW_MIN;
const int GRIPPER_MIN = 1667, GRIPPER_MAX = 2778, GRIPPER_RANGE = GRIPPER_MAX - GRIPPER_MIN;

// TPD = Ticks Per Degree
const int
  BASE_TPD = (BASE_RANGE) / 180,
  SHOULDER_TPD = (SHOULDER_RANGE) / 180,
  ELBOW_TPD = (ELBOW_RANGE) / 180,
  GRIPPER_TPD = (GRIPPER_RANGE) / 180;

int msPerDeg = 10;
unsigned long justNow = 0;
volatile int
  totalTime = 0,
  baseTime = (BASE_RANGE)/2 + BASE_MIN,
  shoulderTime = (SHOULDER_RANGE)/2 + SHOULDER_MIN,
  elbowTime = (ELBOW_RANGE)/2 + ELBOW_MIN,
  gripperTime = (GRIPPER_RANGE)/2 + GRIPPER_MIN;

volatile int
  baseTarget = (BASE_RANGE)/2 + BASE_MIN,
  shoulderTarget = (SHOULDER_RANGE)/2 + SHOULDER_MIN,
  elbowTarget = (ELBOW_RANGE)/2 + ELBOW_MIN,
  gripperTarget = (GRIPPER_RANGE)/2 + GRIPPER_MIN;
volatile int part = 0;


int parse3(const String *s) {
  if (!s) return -1;
  if (s->length() != 3) return -1;
  if (!isDigit(s->charAt(0)) || !isDigit(s->charAt(1)) || !isDigit(s->charAt(2))) return -1;
  return (s->charAt(0) - '0') * 100 + (s->charAt(1) - '0') * 10 + (s->charAt(2) - '0');
}

int stepTowards(int current, int target, int stepSize) {
  if (current < target) {
    current += stepSize;
    if (current > target) current = target;
  } else if (current > target) {
    current -= stepSize;
    if (current < target) current = target; // so it dont bounce
  }
  //Serial.println(current);
  return current;
}

void smoothen() {
  unsigned long now = millis();
  if (now - justNow >= (unsigned long)msPerDeg) {
    
    // ATOMIC BLOCK: Pause interrupts so the 16-bit variables don't corrupt
    //cli();
    
    baseTime = stepTowards(baseTime, baseTarget, BASE_TPD);
    shoulderTime = stepTowards(shoulderTime, shoulderTarget, SHOULDER_TPD);
    elbowTime = stepTowards(elbowTime, elbowTarget, ELBOW_TPD);
    gripperTime = stepTowards(gripperTime, gripperTarget, GRIPPER_TPD);
    
    //sei(); // Resume timer
    
    justNow = now;
  }
}

void homeAll() {
  baseTarget = (BASE_RANGE)/2 + BASE_MIN;
  shoulderTarget = (SHOULDER_RANGE)/2 + SHOULDER_MIN;
  elbowTarget = (ELBOW_RANGE)/2 + ELBOW_MIN;
  gripperTarget = (GRIPPER_RANGE)/2 + GRIPPER_MIN;
}

void setup() {
  Serial.begin(115200);
  cli();
  DDRC |= (1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3);
  PORTC &= ~((1 << PC0) | (1 << PC1) | (1 << PC2) | (1 << PC3));
  TCCR1A = 0b00000000; // ctc mode, top OCR1A
  TCNT1 = 0;
  OCR1A = 1000;
  TIMSK1 |= (1 << OCIE1A);
  TCCR1B = 0b00001010; // prescaler 8
  justNow = millis();
  sei();
}

ISR(TIMER1_COMPA_vect) { // trigger once it sends the pulse to a part
  //TCNT1 = 0;
  switch (part) {
    case 0: // Base
      PORTC |= BASE_MASK;
      OCR1A = baseTime;
      totalTime += baseTime;
      part = 1;
      break;

    case 1: // Shoulder
      PORTC &= ~BASE_MASK;
      PORTC |= SHOULDER_MASK;
      OCR1A = shoulderTime;
      totalTime += shoulderTime;
      part = 2;
      break;

    case 2: // Elbow
      PORTC &= ~SHOULDER_MASK;
      PORTC |= ELBOW_MASK;
      OCR1A = elbowTime;
      totalTime += elbowTime;
      part = 3;
      break;

    case 3: // Gripper
      PORTC &= ~ELBOW_MASK;
      PORTC |= GRIPPER_MASK;
      OCR1A = gripperTime;
      totalTime += gripperTime;
      part = 4;
      break;

    case 4:
      PORTC &= ~GRIPPER_MASK;
      // make sure every period 20ms, force unsigned math
      OCR1A = 40000U - (baseTime + shoulderTime + elbowTime + gripperTime);
      totalTime = 0;
      part = 0;
      break;

  }
}

void loop() {
  smoothen();
  if (!Serial.available()) return;

  // Reads a string with the command until newline
  String cmd = Serial.readStringUntil('\n');
  cmd.trim(); // Remove any extra whitespace
  if (!cmd.length()) return; // didn't read anything

  // Handle the home command
  if (cmd == "H") {
    Serial.println("Homing all servos...");
    homeAll();
    return;
  }

  // All subsequent commands need to have 4 characters
  if (cmd.length() != 4) {
    Serial.println("ERROR: Command is not 4 characters long");
    return;
  }

  // c is now the command character
  char c = cmd.charAt(0);
  // val is the numerical value of the argument
  long val = parse3(&cmd.substring(1));
  if (val < 0) { 
    Serial.println("ERROR: Argument not valid");
    return;
  }

  // Vddd sets velocity as ms per degree
  if (c == 'V') {
    Serial.print("Setting velocity to ");
    Serial.println(val);
    msPerDeg = val;
    return;
  } else if (c == 'B') {
    Serial.print("Moving base to ");
    Serial.println(val);
    val = constrain(val, 0, 180);
    baseTarget = long(val * (BASE_MAX - BASE_MIN)) / 180 + BASE_MIN;
    // baseTarget = constrain(baseTarget, BASE_MIN, BASE_MAX);
    //Serial.println(baseTarget);
    return;
  } else if (c == 'S') {
    Serial.print("Moving shoulder to ");
    Serial.println(val);
    val = constrain(val, 0, 180);
    shoulderTarget = long(val * (SHOULDER_MAX - SHOULDER_MIN)) / 180 + SHOULDER_MIN;
    return;
  } else if (c == 'E') {
    Serial.print("Moving elbow to ");
    Serial.println(val);
    val = constrain(val, 0, 180);
    elbowTarget = long(val * (ELBOW_MAX - ELBOW_MIN)) / 180 + ELBOW_MIN;
    return;
  } else if (c == 'G') {
    Serial.print("Moving gripper to ");
    Serial.println(val);
    val = constrain(val, 0, 180);
    gripperTarget = long(val * (GRIPPER_MAX - GRIPPER_MIN)) / 180 + GRIPPER_MIN;
    return;
  } else {
    Serial.println("ERROR: Unknown command");
    return;
  }
}
