#include <Arduino.h>
#include "TB6612FNG.h"

// =============================================================================
// TB6612FNG Pin Configuration (ESP32 DOIT DevKit V1)
// Adjust these pins according to your actual wiring
// =============================================================================
// Motor A (Left Motor)
const uint8_t PIN_AIN1  = 18;
const uint8_t PIN_AIN2  = 19;
const uint8_t PIN_PWMA  = 23;
const uint8_t PWM_CH_A  = 0;

// Motor B (Right Motor)
const uint8_t PIN_BIN1  = 32;
const uint8_t PIN_BIN2  = 33;
const uint8_t PIN_PWMB  = 25;
const uint8_t PWM_CH_B  = 1;

// Standby Pin
const int8_t  PIN_STBY  = 22;

// Set to true if a motor runs backwards when commanded forward
const bool INVERT_MOTOR_A = false;
const bool INVERT_MOTOR_B = false;

// Create motor driver instance
TB6612FNG rover(
    PIN_AIN1, PIN_AIN2, PIN_PWMA, PWM_CH_A,
    PIN_BIN1, PIN_BIN2, PIN_PWMB, PWM_CH_B,
    PIN_STBY,
    INVERT_MOTOR_A, INVERT_MOTOR_B
);

// Default driving speed (0 - 255)
uint8_t currentSpeed = 180;

void printHelp() {
    Serial.println("\n--- Rover TB6612FNG Serial Control ---");
    Serial.println("  [w] : Forward");
    Serial.println("  [s] : Backward");
    Serial.println("  [a] : Turn Left");
    Serial.println("  [d] : Turn Right");
    Serial.println("  [q] : Spin Left (In-place)");
    Serial.println("  [e] : Spin Right (In-place)");
    Serial.println("  [ ] : Space to Coast / Stop");
    Serial.println("  [b] : Short Brake");
    Serial.println("  [+] : Speed Up (+20)");
    Serial.println("  [-] : Slow Down (-20)");
    Serial.println("  [t] : Run Automated Demo Test");
    Serial.println("  [h] : Show this Help");
    Serial.print("Current Speed: ");
    Serial.println(currentSpeed);
    Serial.println("--------------------------------------\n");
}

void runDemo() {
    Serial.println("[DEMO] Starting motor test sequence...");
    
    Serial.println("[DEMO] Forward");
    rover.forward(currentSpeed);
    delay(1500);

    Serial.println("[DEMO] Brake");
    rover.brake();
    delay(500);

    Serial.println("[DEMO] Backward");
    rover.backward(currentSpeed);
    delay(1500);

    Serial.println("[DEMO] Coast Stop");
    rover.coast();
    delay(500);

    Serial.println("[DEMO] Spin Left");
    rover.spinLeft(currentSpeed);
    delay(1000);

    Serial.println("[DEMO] Spin Right");
    rover.spinRight(currentSpeed);
    delay(1000);

    Serial.println("[DEMO] Test complete. Stopping.");
    rover.brake();
}

void setup() {
    Serial.begin(115200);
    delay(1000);

    Serial.println("\nInitializing TB6612FNG Motor Controller...");
    // 20kHz PWM frequency for silent motor operation, 8-bit resolution (0-255)
    rover.begin(20000, 8);

    printHelp();
}

void loop() {
    if (Serial.available() > 0) {
        char cmd = Serial.read();

        switch (cmd) {
            case 'w':
            case 'W':
                Serial.printf("Command: FORWARD (Speed: %d)\n", currentSpeed);
                rover.forward(currentSpeed);
                break;

            case 's':
            case 'S':
                Serial.printf("Command: BACKWARD (Speed: %d)\n", currentSpeed);
                rover.backward(currentSpeed);
                break;

            case 'a':
            case 'A':
                Serial.printf("Command: TURN LEFT (Speed: %d)\n", currentSpeed);
                rover.turnLeft(currentSpeed);
                break;

            case 'd':
            case 'D':
                Serial.printf("Command: TURN RIGHT (Speed: %d)\n", currentSpeed);
                rover.turnRight(currentSpeed);
                break;

            case 'q':
            case 'Q':
                Serial.printf("Command: SPIN LEFT (Speed: %d)\n", currentSpeed);
                rover.spinLeft(currentSpeed);
                break;

            case 'e':
            case 'E':
                Serial.printf("Command: SPIN RIGHT (Speed: %d)\n", currentSpeed);
                rover.spinRight(currentSpeed);
                break;

            case ' ':
                Serial.println("Command: COAST STOP");
                rover.coast();
                break;

            case 'b':
            case 'B':
                Serial.println("Command: BRAKE");
                rover.brake();
                break;

            case '+':
            case '=':
                if (currentSpeed <= 235) currentSpeed += 20;
                else currentSpeed = 255;
                Serial.printf("Speed increased to: %d\n", currentSpeed);
                break;

            case '-':
            case '_':
                if (currentSpeed >= 40) currentSpeed -= 20;
                else currentSpeed = 20;
                Serial.printf("Speed decreased to: %d\n", currentSpeed);
                break;

            case 't':
            case 'T':
                runDemo();
                break;

            case 'h':
            case 'H':
                printHelp();
                break;

            case '\r':
            case '\n':
                // Ignore newlines
                break;

            default:
                Serial.printf("Unknown command '%c'. Press 'h' for help.\n", cmd);
                break;
        }
    }
}