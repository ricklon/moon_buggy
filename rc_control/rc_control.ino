#include <Bluepad32.h>
#include <ESP32Servo.h>

// Pin Configuration
// Using D9 and D10 which correspond to GPIO 9 and GPIO 10 on ESP32
const int LEFT_MOTOR_PIN = D9;    // D9 on ESP32
const int RIGHT_MOTOR_PIN = D10;  // D10 on ESP32

// Configuration
const int DEAD_ZONE = 50;        // Joystick dead zone
const bool INVERT_LEFT_MOTOR = false;
const bool INVERT_RIGHT_MOTOR = false;
const unsigned long UPDATE_INTERVAL = 20;  // Servo update interval (ms)
const unsigned long TURN_DURATION = 250;   // 180-degree turn duration (ms)

// Servo configuration
const int SERVO_MIN_US = 1000;   // Minimum pulse width in microseconds
const int SERVO_MAX_US = 2000;   // Maximum pulse width in microseconds
const int SERVO_NEUTRAL = 1500;  // Neutral position pulse width

// Constants
const uint16_t LEFT_BUMPER_MASK = 0x0020;

// Global variables
ControllerPtr myControllers[BP32_MAX_GAMEPADS];
Servo leftServo;
Servo rightServo;
int leftSpeed = SERVO_NEUTRAL;
int rightSpeed = SERVO_NEUTRAL;
unsigned long lastUpdate = 0;
bool controllerConnected = false;
bool turning = false;
unsigned long turnStartTime = 0;

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
    leftSpeed = SERVO_NEUTRAL;
    rightSpeed = SERVO_NEUTRAL;
    updateMotors();
    Serial.println("Motors stopped.");
}

int applyDeadZone(int value) {
    return (abs(value) < DEAD_ZONE) ? 0 : value;
}

void processGamepad(ControllerPtr ctl) {
    if (ctl->buttons() & LEFT_BUMPER_MASK) {
        if (!turning) {
            turning = true;
            turnStartTime = millis();
            leftSpeed = INVERT_LEFT_MOTOR ? SERVO_MAX_US : SERVO_MIN_US;
            rightSpeed = INVERT_RIGHT_MOTOR ? SERVO_MIN_US : SERVO_MAX_US;
            Serial.println("180-degree turn started.");
        }
    } else if (!turning) {
        int x = applyDeadZone(ctl->axisX());
        int y = applyDeadZone(ctl->axisY());
        
        // Calculate motor speeds
        leftSpeed = y + x;
        rightSpeed = y - x;
        
        // Apply motor inversion
        if (INVERT_LEFT_MOTOR) leftSpeed = -leftSpeed;
        if (INVERT_RIGHT_MOTOR) rightSpeed = -rightSpeed;
        
        // Map from controller range (-512 to 512) to servo range
        leftSpeed = map(leftSpeed, -512, 512, SERVO_MIN_US, SERVO_MAX_US);
        rightSpeed = map(rightSpeed, -512, 512, SERVO_MIN_US, SERVO_MAX_US);
        
        // Debug output
        Serial.printf("Joystick X: %d, Y: %d\n", x, y);
        Serial.printf("Raw speeds - Left: %d, Right: %d\n", leftSpeed, rightSpeed);
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
    leftServo.writeMicroseconds(leftSpeed);
    rightServo.writeMicroseconds(rightSpeed);
    Serial.printf("Updating servos: Left=%d, Right=%d\n", leftSpeed, rightSpeed);
}

void setup() {
    Serial.begin(115200);
    
    // Initialize Bluepad32
    BP32.setup(&onConnectedController, &onDisconnectedController);
    
    // Allow allocation of all timers
    ESP32PWM::allocateTimer(0);
    ESP32PWM::allocateTimer(1);
    ESP32PWM::allocateTimer(2);
    ESP32PWM::allocateTimer(3);
    
    // Initialize servos
    leftServo.setPeriodHertz(50);    // Standard 50hz servo
    rightServo.setPeriodHertz(50);   // Standard 50hz servo
    
    // Attach servos to their respective pins
    if (leftServo.attach(LEFT_MOTOR_PIN, SERVO_MIN_US, SERVO_MAX_US)) {
        Serial.println("Left servo attached successfully");
    } else {
        Serial.println("Error attaching left servo!");
    }
    
    if (rightServo.attach(RIGHT_MOTOR_PIN, SERVO_MIN_US, SERVO_MAX_US)) {
        Serial.println("Right servo attached successfully");
    } else {
        Serial.println("Error attaching right servo!");
    }
    
    // Set to neutral position
    stopMotors();
    
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
