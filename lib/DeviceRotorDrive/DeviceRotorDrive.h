
#ifndef DEVICEROTORDRIVE_H
#define DEVICEROTORDRIVE_H

#include <Arduino.h>

class DeviceRotorDrive {
  private:
    int _pinIN3;
    int _pinIN4;
    int _pinENB;

  public:
    DeviceRotorDrive();
    DeviceRotorDrive(int pinIN3, int pinIN4, int pinENB);

    void stop(void);
    void rotateClockwise(void);
    void rotateCounterClockwise(void);
};

#endif // DEVICEROTORDRIVE_H