/*
 * Organization: MTL Horizon
 * Modified by: Adithya Selvakumar et al.
 * 
 * This program controls a Remotely Operated Vehicle (ROV) using an Arduino Mega.
 * It interfaces with joysticks, potentiometers, and buttons for input, and
 * outputs to a Pololu Maestro servo controller for motor and servo control.
 */

#include <PololuMaestro.h>
#include <Servo.h>

/* Variable Declaration */

/* INPUTS */

// Joystick Configuration
int JS_PIN_1 = 0; // Surge Forward <-> Surge Backward
int JS_PIN_2 = 3; // Turn Left <-> Turn Right
int JS_PIN_3 = 2; // Heave Up <-> Heave Down
int JS_PIN_4 = 1; // Sway Left <-> Sway Right

int SW_PIN = 47; // Swaps the forward and sway command

// Potentiometer Configuration
int POT_PIN_1 = 5; // Input for Camera Servo
int POT_PIN_2 = 4; // Input for Speed Control
int POT_PIN_3 = 10; // Gripper 1 Open/Close
int POT_PIN_4 = 8; // Gripper 1 Rotation
int POT_PIN_5 = 9; // Gripper 2 Open/Close
int POT_PIN_6 = 11; // Flashlight On/Off

// Variables to store joystick and potentiometer values
int jsVal1 = 0, jsVal2 = 0, jsVal3 = 0, jsVal4 = 0;
int potVal1 = 0, potVal2 = 0, potVal3 = 0, potVal4 = 0, potVal5 = 0, potVal6 = 0;

// Joystick center calibration values
int JS_CENTER_1 = 510, JS_CENTER_2 = 513, JS_CENTER_3 = 515, JS_CENTER_4 = 506;

// Axis direction multipliers (used for joystick inversion)
int AXIS_1 = 1, AXIS_2 = 1, AXIS_3 = 1, AXIS_4 = 1;

int POT_RANGE = 128; // Potentiometer range for mapping

// Button Configuration
int BTN_PIN_5 = 52; // Vertical Booster

// LED and SOS Configuration
int LED_PIN_1 = 42; // BLINK = INITIALIZING! ON = SETUP COMPLETE! (GREEN)
int LED_PIN_2 = 40; // BLINK = INITIALIZING! (WHITE)
int LED_PIN_3 = 34; // ON = FORWARD/SWAY SWITCH (BLUE)
int LED_PIN_4 = 38; // ON = FLASHLIGHT LIT! (YELLOW)
int LED_PIN_5 = 36; // ON = LEAK! (RED)

int SOS_PIN = 30; // Input pin for leak sensor

/* OUTPUTS */

// Motor Configuration
const float MOTOR_DEFAULT = 1500; // Default PWM value for motors (stopped)
float horizSpeed = 100; // Horizontal speed
float vertSpeed = 100; // Vertical speed
float MIN_HORIZ_SPEED = 25, MAX_HORIZ_SPEED = 176; // Speed limits for horizontal movement
float MIN_VERT_SPEED = 25, MAX_VERT_SPEED = 176; // Speed limits for vertical movement

// Command variables for different movement directions
float forwardCommand = MOTOR_DEFAULT;
float turnCommand = MOTOR_DEFAULT;
float vertCommand = MOTOR_DEFAULT;
float swayCommand = MOTOR_DEFAULT;

// Motor output values
float horizMtr1Val = MOTOR_DEFAULT;
float horizMtr2Val = MOTOR_DEFAULT;
float horizMtr3Val = MOTOR_DEFAULT;
float horizMtr4Val = MOTOR_DEFAULT;
float vertMtr1Val = MOTOR_DEFAULT;
float vertMtr2Val = MOTOR_DEFAULT;

// Servo Configuration
int SERVO_UPPER_BOUND = 2000;
int SERVO_LOWER_BOUND = 10000;
int FLASHLIGHT_UPPER_BOUND = 3000;
int FLASHLIGHT_LOWER_BOUND = 0;

// Servo output values and states
int servo1Val = SERVO_LOWER_BOUND;
int servo2Val = SERVO_LOWER_BOUND;
int servo3Val = SERVO_LOWER_BOUND;
int flashlightVal = FLASHLIGHT_LOWER_BOUND;
String servo1State = "CLOSED";
String servo2State = "CLOSED";
String servo3State = "CLOSED";
String flashlightState = "OFF";

// Maestro channel configuration
int HORIZ_MTR1_CHANNEL = 10;
int HORIZ_MTR2_CHANNEL = 7;
int HORIZ_MTR3_CHANNEL = 8;
int HORIZ_MTR4_CHANNEL = 9;
int VERT_MTR1_CHANNEL = 6;
int VERT_MTR2_CHANNEL = 11;
int SERVO1_CHANNEL = 14;
int SERVO2_CHANNEL = 15;
int SERVO3_CHANNEL = 12;
int FLASHLIGHT_CHANNEL = 13;

// Deadband values for joystick inputs
float FORWARD_DEADBAND = 0.5;
float TURN_DEADBAND = 0.5;
float VERT_DEADBAND = 0.5;
float SWAY_DEADBAND = 0.5;

// Speed multipliers for turning and sway movements
float TURN_SPEED_MULTIPLIER = 0.5;
float turnSpeed = horizSpeed * TURN_SPEED_MULTIPLIER;

float SWAY_SPEED_MULTIPLIER = 0.9;
float swaySpeed = horizSpeed * SWAY_SPEED_MULTIPLIER;

/* Object Declaration */

MiniMaestro maestro(Serial2); // Create Maestro object for servo control

void setup()
{
  // Initialize serial communication
  Serial.begin(9600);  // For communication with Serial Monitor
  Serial2.begin(9600); // For communication with Maestro

  // Initialize all motors to default (stopped) position
  maestro.setTarget(HORIZ_MTR1_CHANNEL, MOTOR_DEFAULT * 4.0);
  maestro.setTarget(HORIZ_MTR2_CHANNEL, MOTOR_DEFAULT * 4.0);
  maestro.setTarget(HORIZ_MTR3_CHANNEL, MOTOR_DEFAULT * 4.0);
  maestro.setTarget(HORIZ_MTR4_CHANNEL, MOTOR_DEFAULT * 4.0);
  maestro.setTarget(VERT_MTR1_CHANNEL, MOTOR_DEFAULT * 4.0);
  maestro.setTarget(VERT_MTR2_CHANNEL, MOTOR_DEFAULT * 4.0);

  // Set up LED pins as outputs
  pinMode(LED_PIN_1, OUTPUT);
  pinMode(LED_PIN_2, OUTPUT);
  pinMode(LED_PIN_3, OUTPUT);
  pinMode(LED_PIN_4, OUTPUT);
  pinMode(LED_PIN_5, OUTPUT);
  
  // Set up input pins
  pinMode(SOS_PIN, INPUT);
  pinMode(SW_PIN, INPUT_PULLUP);
  pinMode(BTN_PIN_5, INPUT_PULLUP);

  // Check for leaks
  SOS_CHECK();

  // Initialization sequence with LED blinking
  digitalWrite(LED_PIN_1, HIGH);
  digitalWrite(LED_PIN_2, HIGH);
  delay(1000);

  for (int i = 1; i <= 3; i++) {
    digitalWrite(LED_PIN_1, LOW);
    digitalWrite(LED_PIN_2, LOW);
    delay(1000);
    digitalWrite(LED_PIN_1, HIGH);
    digitalWrite(LED_PIN_2, HIGH);
    delay(1000);
  }
}

void loop()
{
  // Check for leaks
  SOS_CHECK();

  // Read joystick values
  jsVal1 = analogRead(JS_PIN_1);
  jsVal2 = analogRead(JS_PIN_2);
  jsVal3 = analogRead(JS_PIN_3);
  jsVal4 = analogRead(JS_PIN_4);
  
  // Check if forward/sway switch is activated
  if (digitalRead(SW_PIN) == HIGH) {
    // Invert axes for swapped control mode
    AXIS_1 = -1;
    AXIS_2 = 1;
    AXIS_4 = -1;
    digitalWrite(LED_PIN_3, HIGH);
  } else {
    // Normal control mode
    AXIS_1 = 1;
    AXIS_2 = 1;
    AXIS_4 = 1;
    digitalWrite(LED_PIN_3, LOW);
  }
  
  // Read potentiometer values
  potVal1 = analogRead(POT_PIN_1);
  potVal2 = analogRead(POT_PIN_2);
  potVal3 = analogRead(POT_PIN_3);
  potVal4 = analogRead(POT_PIN_4);
  potVal5 = analogRead(POT_PIN_5);
  potVal6 = analogRead(POT_PIN_6);

  // Check if vertical booster is activated
  if (digitalRead(BTN_PIN_5) == LOW) {
    // Vertical booster mode
    horizSpeed = 0;
    vertSpeed = 272;
    turnSpeed = 0;
    swaySpeed = 0;
  } else {
    // Normal speed control
    horizSpeed = map(potVal2, 0, 1023, MIN_HORIZ_SPEED, MAX_HORIZ_SPEED);
    vertSpeed = map(potVal2, 0, 1023, MIN_VERT_SPEED, MAX_VERT_SPEED);
    turnSpeed = horizSpeed * TURN_SPEED_MULTIPLIER;
    swaySpeed = horizSpeed * SWAY_SPEED_MULTIPLIER;
  }
      
  // Map joystick values to motor commands
  forwardCommand = map(jsVal1, JS_CENTER_1 - POT_RANGE, JS_CENTER_1 + POT_RANGE, -horizSpeed * AXIS_1, horizSpeed * AXIS_1);
  turnCommand = map(jsVal2, JS_CENTER_2 - POT_RANGE, JS_CENTER_2 + POT_RANGE, -turnSpeed * AXIS_2, turnSpeed * AXIS_2);
  vertCommand = map(jsVal3, JS_CENTER_3 - POT_RANGE, JS_CENTER_3 + POT_RANGE, -vertSpeed * AXIS_3, vertSpeed * AXIS_3);
  swayCommand = map(jsVal4, JS_CENTER_4 - POT_RANGE, JS_CENTER_4 + POT_RANGE, -swaySpeed * AXIS_4, swaySpeed * AXIS_4);

  // Apply deadbands to commands
  if (abs(forwardCommand) < horizSpeed * FORWARD_DEADBAND) forwardCommand = 0;
  if (abs(turnCommand) < turnSpeed * TURN_DEADBAND) turnCommand = 0;
  if (abs(vertCommand) < vertSpeed * VERT_DEADBAND) vertCommand = 0;
  if (abs(swayCommand) < swaySpeed * SWAY_DEADBAND) swayCommand = 0;

  // Calculate motor values based on commands
  horizMtr1Val = MOTOR_DEFAULT - forwardCommand - turnCommand - swayCommand;
  horizMtr2Val = MOTOR_DEFAULT + forwardCommand - turnCommand + swayCommand;
  horizMtr3Val = MOTOR_DEFAULT + forwardCommand + turnCommand - swayCommand;
  horizMtr4Val = MOTOR_DEFAULT - forwardCommand + turnCommand + swayCommand;
  vertMtr1Val = MOTOR_DEFAULT + vertCommand;
  vertMtr2Val = MOTOR_DEFAULT + vertCommand;

  // Map potentiometer values to servo positions
  servo1Val = map(potVal3, 0, 1023, SERVO_LOWER_BOUND, SERVO_UPPER_BOUND);
  servo2Val = map(potVal4, 0, 1023, SERVO_LOWER_BOUND, SERVO_UPPER_BOUND);
  servo3Val = map(potVal5, 0, 1023, SERVO_LOWER_BOUND, SERVO_UPPER_BOUND);
  flashlightVal = map(potVal6, 0, 1023, FLASHLIGHT_LOWER_BOUND, FLASHLIGHT_UPPER_BOUND);

  // Control flashlight LED indicator
  if (flashlightVal >= ((FLASHLIGHT_UPPER_BOUND - FLASHLIGHT_LOWER_BOUND) / 2)) {
    digitalWrite(LED_PIN_4, HIGH);
  } else {
    digitalWrite(LED_PIN_4, LOW);
  }

  // Map servo positions to degree values for display
  servo1State = map(potVal3, 0, 1023, 0, 180);
  servo2State = map(potVal4, 0, 1023, 0, 180);
  servo3State = map(potVal5, 0, 1023, 0, 180);
  flashlightState = map(potVal6, 0, 1023, 0, 255);
  
  // Set motor targets on Maestro
  maestro.setTarget(HORIZ_MTR1_CHANNEL, horizMtr1Val * 4);
  maestro.setTarget(HORIZ_MTR2_CHANNEL, horizMtr2Val * 4);
  maestro.setTarget(HORIZ_MTR3_CHANNEL, horizMtr3Val * 4);
  maestro.setTarget(HORIZ_MTR4_CHANNEL, horizMtr4Val * 4);
  maestro.setTarget(VERT_MTR1_CHANNEL, vertMtr1Val * 4);
  maestro.setTarget(VERT_MTR2_CHANNEL, vertMtr2Val * 4);

  // Set servo targets on Maestro
  maestro.setTarget(SERVO1_CHANNEL, servo1Val);
  maestro.setTarget(SERVO2_CHANNEL, servo2Val);
  maestro.setTarget(SERVO3_CHANNEL, servo3Val);
  maestro.setTarget(FLASHLIGHT_CHANNEL, flashlightVal);
  
  // Print debugging information to Serial Monitor
  printDebugInfo();
}

// Function to check for leaks and update LED
void SOS_CHECK() {
  digitalWrite(LED_PIN_5, digitalRead(SOS_PIN));

  if (digitalRead(SOS_PIN) == 1) {
    Serial.println("LEAK DETECTED");
  }
}

// Function to print debugging information to Serial Monitor
void printDebugInfo() {
  Serial.println("----TROUBLESHOOTING MENU----");
  Serial.println();
  Serial.println("--AXIS INPUT VALUES--");
  Serial.print("J1: "); Serial.println(jsVal1);
  Serial.print("J2: "); Serial.println(jsVal2);
  Serial.print("J3: "); Serial.println(jsVal3);
  Serial.print("J4: "); Serial.println(jsVal4);
  Serial.print("P1: "); Serial.println(potVal1);
  Serial.print("P2: "); Serial.println(potVal2);
  Serial.print("P3: "); Serial.println(potVal3);
  Serial.print("P4: "); Serial.println(potVal4);
  Serial.print("P5: "); Serial.println(potVal5);
  Serial.print("P6: "); Serial.println(potVal6);
  Serial.println();
  Serial.println("--MOTOR OUTPUT VALUES--");
  Serial.print("M1: "); Serial.println(horizMtr1Val);
  Serial.print("M2: "); Serial.println(horizMtr2Val);
  Serial.print("M3: "); Serial.println(horizMtr3Val);
  Serial.print("M4: "); Serial.println(horizMtr4Val);
  Serial.print("V1: "); Serial.println(vertMtr1Val);
  Serial.print("V2: "); Serial.println(vertMtr2Val);
  Serial.println();
  Serial.println("--SERVO STATE VALUES--");
  Serial.print("1: "); Serial.println(servo1State);
  Serial.print("2: "); Serial.println(servo2State);
  Serial.print("3: "); Serial.println(servo3State);
  Serial.print("LIGHT: "); Serial.println(flashlightState);
  Serial.println();
  Serial.println("--SERVO OUTPUT VALUES--");
  Serial.print("1: "); Serial.println(servo1Val);
  Serial.print("2: "); Serial.println(servo2Val);
  Serial.print("3: "); Serial.println(servo3Val);
  Serial.print("LIGHT: "); Serial.println(flashlightVal);
  Serial.println();
}