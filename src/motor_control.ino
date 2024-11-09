#define STEP_PIN_1 2  // Motor 1 STEP Pin
#define DIR_PIN_1 3   // Motor 1 DIR Pin

#define STEP_PIN_2 4  // Motor 2 STEP Pin
#define DIR_PIN_2 5   // Motor 2 DIR Pin

#define STEP_PIN_3 6  // Motor 3 STEP Pin
#define DIR_PIN_3 7   // Motor 3 DIR Pin

// Constants for motor step angle and movement limits
const float stepAngle = 1.8;  // 1.8 degrees per step for Nema 17
const float maxAngle = 80.0;  // Maximum angle (in degrees)
const float minAngle = 0.0;   // Minimum angle (in degrees)

// Constants for platform position range
const float platformMax = 25.0;
const float platformMin = -25.0;

// Variables to track current position for each motor
float currentAngle1 = 0.0;  // Motor 1 current angle
float currentAngle2 = 0.0;  // Motor 2 current angle
float currentAngle3 = 0.0;  // Motor 3 current angle

void setup() {
  // Initialize pins for all motors
  pinMode(STEP_PIN_1, OUTPUT);
  pinMode(DIR_PIN_1, OUTPUT);
  pinMode(STEP_PIN_2, OUTPUT);
  pinMode(DIR_PIN_2, OUTPUT);
  pinMode(STEP_PIN_3, OUTPUT);
  pinMode(DIR_PIN_3, OUTPUT);
}

void loop() {
  // Example: Move Motor 1 based on platform position
  moveMotor(1, mapPositionToAngle(-10.0));  // Example platform position
  delay(1000);  // Wait for a second
  
  // Example: Move Motor 2 based on platform position
  moveMotor(2, mapPositionToAngle(10.0));  // Example platform position
  delay(1000);
  
  // Example: Move Motor 3 based on platform position
  moveMotor(3, mapPositionToAngle(20.0));  // Example platform position
  delay(1000);
}

// Function to map platform position to motor angle
float mapPositionToAngle(float platformPosition) {
  // Ensure platform position is within the valid range
  if (platformPosition > platformMax) {
    platformPosition = platformMax;
  } else if (platformPosition < platformMin) {
    platformPosition = platformMin;
  }
  
  // Map the platform position (-25 to 25) to motor angle (0 to 80)
  // Formula to map: (platformPosition + 25) * (maxAngle - minAngle) / (platformMax - platformMin)
  float motorAngle = (platformPosition + 25.0) * (maxAngle - minAngle) / (platformMax - platformMin);
  
  return motorAngle;
}

void moveMotor(int motor, float targetAngle) {
  // Ensure the target angle is within the allowed range
  if (targetAngle > maxAngle) {
    targetAngle = maxAngle;
  } else if (targetAngle < minAngle) {
    targetAngle = minAngle;
  }

  // Calculate the number of steps required to reach the target angle
  int targetSteps = angleToSteps(targetAngle);
  int stepDifference = 0;

  // Select the motor based on input
  switch(motor) {
    case 1:
      stepDifference = targetSteps - angleToSteps(currentAngle1);
      if (stepDifference > 0) {
        digitalWrite(DIR_PIN_1, HIGH);  // Clockwise
      } else {
        digitalWrite(DIR_PIN_1, LOW);   // Counterclockwise
        stepDifference = -stepDifference;
      }
      // Move Motor 1
      for (int i = 0; i < stepDifference; i++) {
        digitalWrite(STEP_PIN_1, HIGH);
        delayMicroseconds(1000);  // Control speed with delay
        digitalWrite(STEP_PIN_1, LOW);
        delayMicroseconds(1000);
      }
      currentAngle1 = targetAngle;  // Update current position for Motor 1
      break;

    case 2:
      stepDifference = targetSteps - angleToSteps(currentAngle2);
      if (stepDifference > 0) {
        digitalWrite(DIR_PIN_2, HIGH);  // Clockwise
      } else {
        digitalWrite(DIR_PIN_2, LOW);   // Counterclockwise
        stepDifference = -stepDifference;
      }
      // Move Motor 2
      for (int i = 0; i < stepDifference; i++) {
        digitalWrite(STEP_PIN_2, HIGH);
        delayMicroseconds(1000);  // Control speed with delay
        digitalWrite(STEP_PIN_2, LOW);
        delayMicroseconds(1000);
      }
      currentAngle2 = targetAngle;  // Update current position for Motor 2
      break;

    case 3:
      stepDifference = targetSteps - angleToSteps(currentAngle3);
      if (stepDifference > 0) {
        digitalWrite(DIR_PIN_3, HIGH);  // Clockwise
      } else {
        digitalWrite(DIR_PIN_3, LOW);   // Counterclockwise
        stepDifference = -stepDifference;
      }
      // Move Motor 3
      for (int i = 0; i < stepDifference; i++) {
        digitalWrite(STEP_PIN_3, HIGH);
        delayMicroseconds(1000);  // Control speed with delay
        digitalWrite(STEP_PIN_3, LOW);
        delayMicroseconds(1000);
      }
      currentAngle3 = targetAngle;  // Update current position for Motor 3
      break;
  }
}

// Convert angle to number of steps
int angleToSteps(float angle) {
  return (int)(angle / stepAngle);  // Convert angle to steps
}
