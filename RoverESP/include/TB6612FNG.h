#ifndef TB6612FNG_H
#define TB6612FNG_H

#include <Arduino.h>

/**
 * @brief Class for controlling a single DC motor via TB6612FNG.
 */
class Motor {
public:
    Motor(uint8_t pinIn1, uint8_t pinIn2, uint8_t pinPWM, uint8_t pwmChannel, bool inverted = false);

    void begin(uint32_t pwmFreq = 20000, uint8_t pwmResolution = 8);
    void drive(int16_t speed);
    void forward(uint8_t speed);
    void backward(uint8_t speed);
    void brake();
    void coast();

private:
    uint8_t _pinIn1;
    uint8_t _pinIn2;
    uint8_t _pinPWM;
    uint8_t _channel;
    bool _inverted;
    uint8_t _resolution;
    uint16_t _maxDuty;

    void setPWM(uint32_t duty);
};

/**
 * @brief Class for controlling a dual-motor driver (TB6612FNG) with STBY support.
 */
class TB6612FNG {
public:
    TB6612FNG(
        uint8_t pinAIN1, uint8_t pinAIN2, uint8_t pinPWMA, uint8_t channelA,
        uint8_t pinBIN1, uint8_t pinBIN2, uint8_t pinPWMB, uint8_t channelB,
        int8_t pinSTBY = -1,
        bool invertA = false, bool invertB = false
    );

    void begin(uint32_t pwmFreq = 20000, uint8_t pwmResolution = 8);
    void enable();
    void disable();

    // Direct differential drive controls
    void drive(int16_t speedLeft, int16_t speedRight);
    void forward(uint8_t speed);
    void backward(uint8_t speed);
    void turnLeft(uint8_t speed);   // gentle turn (one wheel stopped)
    void turnRight(uint8_t speed);  // gentle turn (one wheel stopped)
    void spinLeft(uint8_t speed);   // point turn (opposite directions)
    void spinRight(uint8_t speed);  // point turn (opposite directions)
    void brake();
    void coast();

    Motor motorA;
    Motor motorB;

private:
    int8_t _pinSTBY;
};

#endif // TB6612FNG_H
