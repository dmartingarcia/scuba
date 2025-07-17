#include <Arduino.h>

/*
IBT-2 Motor Control Board driven.

Connection to the IBT-2 board:
IBT-2 pin 1 (RPWM) to PWM
IBT-2 pin 2 (LPWM) to PWM
IBT-2 pins 3 (R_EN), 4 (L_EN), 7 (VCC) to 5V pin
IBT-2 pin 8 (GND) to GND
IBT-2 pins 5 (R_IS) and 6 (L_IS) not connected
*/

#define AGUA_RPWM_Output 13 // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
#define AGUA_LPWM_Output 12 // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
#define AGUA_R_ENABLE 10 // Not used in this example, but can be connected to Arduino pin 8 if needed
#define AGUA_L_ENABLE 11 // Not used in this example, but can be connected to Arduino pin 7 if needed
#define MOVIMIENTO_RPWM_Output 3 // Arduino PWM output pin 5; connect to IBT-2 pin 1 (RPWM)
#define MOVIMIENTO_LPWM_Output 2 // Arduino PWM output pin 6; connect to IBT-2 pin 2 (LPWM)
#define MOVIMIENTO_R_ENABLE 5 // Not used in this example, but can be connected to Arduino pin 8 if needed
#define MOVIMIENTO_L_ENABLE 4 // Not used in this example, but can be connected to Arduino pin 7 if needed

void setup()
{
  pinMode(MOVIMIENTO_RPWM_Output, OUTPUT);
  pinMode(MOVIMIENTO_LPWM_Output, OUTPUT);
  pinMode(MOVIMIENTO_R_ENABLE, OUTPUT);
  pinMode(MOVIMIENTO_L_ENABLE, OUTPUT);
  pinMode(AGUA_RPWM_Output, OUTPUT);
  pinMode(AGUA_LPWM_Output, OUTPUT);
  pinMode(AGUA_R_ENABLE, OUTPUT);
  pinMode(AGUA_L_ENABLE, OUTPUT);

  digitalWrite(MOVIMIENTO_R_ENABLE, HIGH); // Enable right motor
  digitalWrite(MOVIMIENTO_L_ENABLE, HIGH); // Enable left motor
  digitalWrite(MOVIMIENTO_RPWM_Output, LOW); // Initialize right motor PWM to 0
  digitalWrite(MOVIMIENTO_LPWM_Output, LOW); // Initialize left motor PWM to 0
  digitalWrite(AGUA_R_ENABLE, HIGH); // Enable right motor
  digitalWrite(AGUA_L_ENABLE, HIGH); // Enable left motor
  digitalWrite(AGUA_RPWM_Output, LOW); // Initialize right motor PWM to 0
  digitalWrite(AGUA_LPWM_Output, LOW); // Initialize left motor PWM to 0

  Serial.begin(9600); // Initialize serial communication for debugging
  Serial.println("IBT-2 Motor Control Example");
}

int checkSpeed(int speed){
  if(speed > 255){
    speed = 255;
  }

  if(speed < -255){
    speed = -255;
  }
  return speed;
}
// Function to control the movement motor
void controlMovimientoMotor(int speed) {
  // Speed is in the range -255 to 255
  // Positive value for forward rotation, negative for reverse
  speed = checkSpeed(speed);
  digitalWrite(MOVIMIENTO_R_ENABLE, HIGH); // Disable right motor
  digitalWrite(MOVIMIENTO_L_ENABLE, HIGH); // Disable left motor

  if (speed > 0) {
    // Forward rotation
    analogWrite(MOVIMIENTO_RPWM_Output, speed); // Set PWM for forward rotation
    analogWrite(MOVIMIENTO_LPWM_Output, 0);     // No PWM for reverse
  } else if (speed < 0) {
    // Reverse rotation
    analogWrite(MOVIMIENTO_RPWM_Output, 0);         // No PWM for forward
    analogWrite(MOVIMIENTO_LPWM_Output, -speed);    // Set PWM for reverse rotation
  } else {
    // Stop
    analogWrite(MOVIMIENTO_RPWM_Output, 0);
    analogWrite(MOVIMIENTO_LPWM_Output, 0);
  }
}

// Function to control the movement motor
void controlAguaMotor(int speed) {
  // Speed is in the range -255 to 255
  // Positive value for forward rotation, negative for reverse
  speed = checkSpeed(speed);
  digitalWrite(AGUA_R_ENABLE, HIGH); // Enable the motor
  digitalWrite(AGUA_L_ENABLE, HIGH);

  if (speed > 0) {
    // Forward rotation
    analogWrite(AGUA_RPWM_Output, speed); // Set PWM for forward rotation
    analogWrite(AGUA_LPWM_Output, 0);     // No PWM for reverse
  } else if (speed < 0) {
    // Reverse rotation
    analogWrite(AGUA_RPWM_Output, 0);         // No PWM for forward
    analogWrite(AGUA_LPWM_Output, -speed);    // Set PWM for reverse rotation
  } else {
    // Stop
    analogWrite(AGUA_RPWM_Output, 0);
    analogWrite(AGUA_LPWM_Output, 0);
  }
}

// Demo function to test all motor functionality
void loop() {
  Serial.println("Starting motor test sequence - Agua");
  controlAguaMotor(200);
  delay(1000);
  controlAguaMotor(250);
  delay(1000);
  controlAguaMotor(200);
  delay(1000);

  // Gradual test for movement motor from 100% forward to 100% reverse
  Serial.println("Testing movement motor: 100% forward to 100% reverse");
  controlMovimientoMotor(200);
  delay(5000);

  controlAguaMotor(100);
  controlMovimientoMotor(-200);
  delay(5000);

  // Stop movement motor
  Serial.println("Movement motor stopping after gradual test");
  controlMovimientoMotor(0);
  controlAguaMotor(0);
  delay(3000);
}
