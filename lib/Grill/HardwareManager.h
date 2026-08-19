#ifndef GRILL_HARDWARE_H
#define GRILL_HARDWARE_H

#include <GrillConfig.h>
#include <DeviceEncoder.h>
#include <CytronMotorDriver.h>
// #include <Adafruit_MAX31865.h>  
#include <Adafruit_MAX31855.h>
#include <DeviceRotorDrive.h>
#include <GrillMQTT.h>

class HardwareManager {
public:
    HardwareManager(int index, GrillMQTT* mqtt);

    bool setup_devices();
    void reset_encoder(DeviceEncoder* sel_encoder);
    void reset_rotor_encoder();

    GrillMQTT* mqtt;
    CytronMD* drive;
    DeviceEncoder* encoder;
    DeviceEncoder* rotorEncoder;
    DeviceRotorDrive* rotor;
    Adafruit_MAX31855* thermocouple;

private:

    int grillIndex;

};

#endif
