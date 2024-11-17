#include <Bluepad32.h>
#include <ESP32Servo.h>

// Configuration
const int LEFT_MOTOR_PIN = D10;
const int RIGHT_MOTOR_PIN = D9;

const int DEAD_ZONE = 50;                  // Joystick dead zone
const bool INVERT_LEFT_MOTOR = true;       // Adjust based on motor wiring
const bool INVERT_RIGHT_MOTOR = false;     // Adjust based on motor wiring
const unsigned long UPDATE_INTERVAL = 50;  // Motor update interval (ms)
const unsigned long TURN_DURATION = 250;   // 180-degree turn duration (ms)

// ESC calibration constants
const int ESC_CAL_DELAY = 2000;  // Delay for ESC calibration steps
const int STARTUP_DELAY = 3000;  // Initial power-up delay

// Constants
const int NEUTRAL_SPEED = 1500;
const int MIN_SPEED = 1000;
const int MAX_SPEED = 2000;

// Global variables
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
int leftSpeed = NEUTRAL_SPEED;
int rightSpeed = NEUTRAL_SPEED;
unsigned long lastUpdate = 0;
bool controllerConnected = false;
bool turning = false;
unsigned long turnStartTime = 0;
bool escsArmed = false;

// Create servo objects for ESC control
Servo leftESC;
Servo rightESC;

// Function to arm ESCs
void armESCs() {
  Serial.println("Starting ESC arming sequence...");

  // Attach ESCs
  leftESC.attach(LEFT_MOTOR_PIN, MIN_SPEED, MAX_SPEED);
  rightESC.attach(RIGHT_MOTOR_PIN, MIN_SPEED, MAX_SPEED);

  // Initial delay after power up
  delay(STARTUP_DELAY);
  Serial.println("Power-up delay complete");

  // Set to neutral position
  Serial.println("Setting neutral position...");
  leftESC.writeMicroseconds(NEUTRAL_SPEED);
  rightESC.writeMicroseconds(NEUTRAL_SPEED);
  delay(ESC_CAL_DELAY);

  escsArmed = true;
  Serial.println("ESCs armed and ready!");
}

// Function to calculate motor speed with inversion
int calculateMotorSpeed(int inputValue, bool invertMotor, int direction) {
  // direction: 1 for forward, -1 for reverse
  int mappedSpeed;

  // Ensure we map the full range of the input (0-1023) to the full ESC range
  if (direction == 1) {
    // Map 0-1023 to NEUTRAL_SPEED-MAX_SPEED (1500-2000)
    mappedSpeed = map(inputValue, 0, 1023, NEUTRAL_SPEED, MAX_SPEED);
  } else {
    // Map 0-1023 to MIN_SPEED-NEUTRAL_SPEED (1000-1500)
    mappedSpeed = map(inputValue, 0, 1023, MIN_SPEED, NEUTRAL_SPEED);
  }

  if (invertMotor) {
    // Properly invert around neutral point
    return MIN_SPEED + (MAX_SPEED - mappedSpeed);
  }

  return mappedSpeed;
}

// Function to apply dead zone to joystick inputs
int applyDeadZone(int value) {
  return (abs(value) < DEAD_ZONE) ? 0 : value;
}

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      Serial.printf("Controller connected, index=%d\n", i);
      controllerConnected = true;
      break;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      controllerConnected = false;
      stopMotors();
      break;
    }
  }
}

void stopMotors() {
  leftSpeed = NEUTRAL_SPEED;
  rightSpeed = NEUTRAL_SPEED;
  updateMotors();
  Serial.println("Motors stopped.");
}

void processGamepad(ControllerPtr ctl) {
  if (!escsArmed) return;

  static bool continuousTurning = false;

  bool leftBumperPressed = ctl->l1();
  bool rightBumperPressed = ctl->r1();

  // Get full range from triggers (0-1023)
  int throttleValue = ctl->brake();
  int brakeValue = ctl->throttle();

  // Map joystick to full range (-512 to 512)
  int axisX = applyDeadZone(ctl->axisX());
  int axisY = applyDeadZone(ctl->axisY());

  // Handle left bumper tap and hold for left turn (full speed turn)
  if (leftBumperPressed) {
    if (!turning) {
      turning = true;
      turnStartTime = millis();
      // Full speed turn with inversion check
      leftSpeed = INVERT_LEFT_MOTOR ? MAX_SPEED : MIN_SPEED;    // Reverse or forward based on inversion
      rightSpeed = INVERT_RIGHT_MOTOR ? MIN_SPEED : MAX_SPEED;  // Forward or reverse based on inversion
      Serial.println("180-degree left turn started.");
    }
    continuousTurning = true;
  } else if (rightBumperPressed) {
    if (!turning) {
      turning = true;
      turnStartTime = millis();
      // Full speed turn with inversion check
      leftSpeed = INVERT_LEFT_MOTOR ? MIN_SPEED : MAX_SPEED;    // Forward or reverse based on inversion
      rightSpeed = INVERT_RIGHT_MOTOR ? MAX_SPEED : MIN_SPEED;  // Reverse or forward based on inversion
      Serial.println("180-degree right turn started.");
    }
    continuousTurning = true;
  }
  // Handle right trigger for forward movement
  else if (throttleValue > 50) {
    // Map the throttle value to get full forward range
    leftSpeed = map(throttleValue, 0, 1023, NEUTRAL_SPEED, MAX_SPEED);
    rightSpeed = map(throttleValue, 0, 1023, NEUTRAL_SPEED, MAX_SPEED);

    // Apply inversion if needed
    if (INVERT_LEFT_MOTOR) {
      leftSpeed = MIN_SPEED + (MAX_SPEED - leftSpeed);
    }
    if (INVERT_RIGHT_MOTOR) {
      rightSpeed = MIN_SPEED + (MAX_SPEED - rightSpeed);
    }

    Serial.printf("Right trigger pressed: Forward speed set to Left = %d, Right = %d\n", leftSpeed, rightSpeed);
  }
  // Handle left trigger for reverse movement
  else if (brakeValue > 50) {
    // Map the brake value to get full reverse range
    leftSpeed = map(brakeValue, 0, 1023, NEUTRAL_SPEED, MIN_SPEED);
    rightSpeed = map(brakeValue, 0, 1023, NEUTRAL_SPEED, MIN_SPEED);

    // Apply inversion if needed
    if (INVERT_LEFT_MOTOR) {
      leftSpeed = MAX_SPEED - (leftSpeed - MIN_SPEED);
    }
    if (INVERT_RIGHT_MOTOR) {
      rightSpeed = MAX_SPEED - (rightSpeed - MIN_SPEED);
    }

    Serial.printf("Left trigger pressed: Reverse speed set to Left = %d, Right = %d\n", leftSpeed, rightSpeed);
  }
  // If a turn was in progress but no bumper is pressed, stop turning
  else if (turning || continuousTurning) {
    turning = false;
    continuousTurning = false;
    stopMotors();
    Serial.println("Turn completed and reset.");
  }
  // Normal joystick-based control
  else {
    // Arcade drive calculations
    int leftMotorInput = axisY + axisX;
    int rightMotorInput = axisY - axisX;

    // Constrain the combined inputs to avoid exceeding limits
    leftMotorInput = constrain(leftMotorInput, -512, 512);
    rightMotorInput = constrain(rightMotorInput, -512, 512);

    // Map to full ESC range
    leftSpeed = map(leftMotorInput, -512, 512, MIN_SPEED, MAX_SPEED);
    rightSpeed = map(rightMotorInput, -512, 512, MIN_SPEED, MAX_SPEED);

    // Apply inversion
    if (INVERT_LEFT_MOTOR) {
      leftSpeed = MIN_SPEED + (MAX_SPEED - leftSpeed);
    }
    if (INVERT_RIGHT_MOTOR) {
      rightSpeed = MIN_SPEED + (MAX_SPEED - rightSpeed);
    }

    Serial.printf("Joystick control: Left Speed = %d, Right Speed = %d\n", leftSpeed, rightSpeed);
  }

  // Handle turn duration and continuation with full speed turns
  if (turning && (millis() - turnStartTime >= TURN_DURATION)) {
    turning = false;

    if (leftBumperPressed) {
      turning = true;
      turnStartTime = millis();
      leftSpeed = MIN_SPEED;   // Full reverse
      rightSpeed = MAX_SPEED;  // Full forward
      Serial.println("Continuing 180-degree left turn.");
    } else if (rightBumperPressed) {
      turning = true;
      turnStartTime = millis();
      leftSpeed = MAX_SPEED;   // Full forward
      rightSpeed = MIN_SPEED;  // Full reverse
      Serial.println("Continuing 180-degree right turn.");
    } else {
      stopMotors();
      Serial.println("Turn completed and stopped.");
    }
  }
}

void processControllers() {
  if (controllerConnected) {
    for (auto myController : myControllers) {
      if (myController && myController->isConnected() && myController->hasData()) {
        processGamepad(myController);
      }
    }
  } else {
    stopMotors();
  }
}

void updateMotors() {
  if (!escsArmed) return;

  // Update ESCs with new speed values
  leftESC.writeMicroseconds(leftSpeed);
  rightESC.writeMicroseconds(rightSpeed);
  Serial.printf("Updating ESCs: Left Speed=%d, Right Speed=%d\n",
                leftSpeed, rightSpeed);
}

void setup() {
  Serial.begin(115200);

  // Initialize ESP32Servo
  ESP32PWM::allocateTimer(0);
  ESP32PWM::allocateTimer(1);
  ESP32PWM::allocateTimer(2);
  ESP32PWM::allocateTimer(3);

  // Configure ESC settings
  leftESC.setPeriodHertz(50);   // Standard 50hz servo/ESC rate
  rightESC.setPeriodHertz(50);  // Standard 50hz servo/ESC rate

  // Initialize Bluepad32
  BP32.setup(&onConnectedController, &onDisconnectedController);

  // Arm the ESCs
  armESCs();

  Serial.println("Setup complete, waiting for controllers...");
}

void loop() {
  unsigned long currentMillis = millis();

  if (BP32.update()) {
    processControllers();
  }

  if (turning && currentMillis - turnStartTime >= TURN_DURATION) {
    turning = false;
    stopMotors();
    Serial.println("180-degree turn completed. Motors set to neutral.");
  }

  if (currentMillis - lastUpdate >= UPDATE_INTERVAL) {
    lastUpdate = currentMillis;
    updateMotors();
  }

  vTaskDelay(1);
}
