#include "TB6612FNG.h"
#include <esp_arduino_version.h>

Motor::Motor(uint8_t pinIn1, uint8_t pinIn2, uint8_t pinPWM, uint8_t pwmChannel, bool inverted)
    : _pinIn1(pinIn1), _pinIn2(pinIn2), _pinPWM(pinPWM), _channel(pwmChannel),
      _inverted(inverted), _resolution(8), _maxDuty(255) {}

void Motor::begin(uint32_t pwmFreq, uint8_t pwmResolution) {
    _resolution = pwmResolution;
    _maxDuty = (1 << _resolution) - 1;

    pinMode(_pinIn1, OUTPUT);
    pinMode(_pinIn2, OUTPUT);
    digitalWrite(_pinIn1, LOW);
    digitalWrite(_pinIn2, LOW);

#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcAttach(_pinPWM, pwmFreq, _resolution);
    ledcWrite(_pinPWM, 0);
#else
    ledcSetup(_channel, pwmFreq, _resolution);
    ledcAttachPin(_pinPWM, _channel);
    ledcWrite(_channel, 0);
#endif
}

void Motor::setPWM(uint32_t duty) {
    if (duty > _maxDuty) {
        duty = _maxDuty;
    }
#if defined(ESP_ARDUINO_VERSION_MAJOR) && (ESP_ARDUINO_VERSION_MAJOR >= 3)
    ledcWrite(_pinPWM, duty);
#else
    ledcWrite(_channel, duty);
#endif
}

void Motor::drive(int16_t speed) {
    if (_inverted) {
        speed = -speed;
    }

    if (speed > 0) {
        digitalWrite(_pinIn1, HIGH);
        digitalWrite(_pinIn2, LOW);
        setPWM((uint32_t)speed);
    } else if (speed < 0) {
        digitalWrite(_pinIn1, LOW);
        digitalWrite(_pinIn2, HIGH);
        setPWM((uint32_t)(-speed));
    } else {
        coast();
    }
}

void Motor::forward(uint8_t speed) {
    drive((int16_t)speed);
}

void Motor::backward(uint8_t speed) {
    drive(-(int16_t)speed);
}

void Motor::brake() {
    // Short brake: both inputs HIGH with PWM active
    digitalWrite(_pinIn1, HIGH);
    digitalWrite(_pinIn2, HIGH);
    setPWM(_maxDuty);
}

void Motor::coast() {
    // Coast / Stop: both inputs LOW, zero PWM
    digitalWrite(_pinIn1, LOW);
    digitalWrite(_pinIn2, LOW);
    setPWM(0);
}

TB6612FNG::TB6612FNG(
    uint8_t pinAIN1, uint8_t pinAIN2, uint8_t pinPWMA, uint8_t channelA,
    uint8_t pinBIN1, uint8_t pinBIN2, uint8_t pinPWMB, uint8_t channelB,
    int8_t pinSTBY,
    bool invertA, bool invertB
)
    : motorA(pinAIN1, pinAIN2, pinPWMA, channelA, invertA),
      motorB(pinBIN1, pinBIN2, pinPWMB, channelB, invertB),
      _pinSTBY(pinSTBY) {}

void TB6612FNG::begin(uint32_t pwmFreq, uint8_t pwmResolution) {
    if (_pinSTBY >= 0) {
        pinMode(_pinSTBY, OUTPUT);
        digitalWrite(_pinSTBY, HIGH); // Enable driver by default
    }

    motorA.begin(pwmFreq, pwmResolution);
    motorB.begin(pwmFreq, pwmResolution);
}

void TB6612FNG::enable() {
    if (_pinSTBY >= 0) {
        digitalWrite(_pinSTBY, HIGH);
    }
}

void TB6612FNG::disable() {
    if (_pinSTBY >= 0) {
        digitalWrite(_pinSTBY, LOW);
    }
}

void TB6612FNG::drive(int16_t speedLeft, int16_t speedRight) {
    motorA.drive(speedLeft);
    motorB.drive(speedRight);
}

void TB6612FNG::forward(uint8_t speed) {
    motorA.forward(speed);
    motorB.forward(speed);
}

void TB6612FNG::backward(uint8_t speed) {
    motorA.backward(speed);
    motorB.backward(speed);
}

void TB6612FNG::turnLeft(uint8_t speed) {
    motorA.coast();
    motorB.forward(speed);
}

void TB6612FNG::turnRight(uint8_t speed) {
    motorA.forward(speed);
    motorB.coast();
}

void TB6612FNG::spinLeft(uint8_t speed) {
    motorA.backward(speed);
    motorB.forward(speed);
}

void TB6612FNG::spinRight(uint8_t speed) {
    motorA.forward(speed);
    motorB.backward(speed);
}

void TB6612FNG::brake() {
    motorA.brake();
    motorB.brake();
}

void TB6612FNG::coast() {
    motorA.coast();
    motorB.coast();
}
