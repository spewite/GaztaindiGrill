

#include "DeviceRotorDrive.h"

DeviceRotorDrive::DeviceRotorDrive() {
    _pinIN3 = -1;
    _pinIN4 = -1;
    _pinENB = -1;
};

DeviceRotorDrive::DeviceRotorDrive(int pinIN3, int pinIN4, int pinENB) {
    _pinIN3 = pinIN3;
    _pinIN4 = pinIN4;
    _pinENB = pinENB;
    pinMode(_pinIN3, OUTPUT);
    pinMode(_pinIN4, OUTPUT);
    pinMode(_pinENB, OUTPUT);
    analogWrite(_pinENB, 255); // Máxima velocidad
    stop();
};

void DeviceRotorDrive::stop(void) {
    digitalWrite(_pinIN3, LOW);
    digitalWrite(_pinIN4, LOW);
};

void DeviceRotorDrive::rotateCounterClockwise(void) {
    digitalWrite(_pinIN3, HIGH);
    digitalWrite(_pinIN4, LOW);
};

void DeviceRotorDrive::rotateClockwise(void) {
    digitalWrite(_pinIN3, LOW);
    digitalWrite(_pinIN4, HIGH);
};
