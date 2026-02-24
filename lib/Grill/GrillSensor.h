#ifndef GRILL_SENSOR_H
#define GRILL_SENSOR_H

#include <GrillMQTT.h>
#include <HardwareManager.h>
#include <ModeManager.h>
#include <GrillConstants.h>

class GrillSensor {
public:

    GrillSensor(int index, GrillMQTT* mqtt, HardwareManager* hardware, ModeManager* modeManager);

    // ----------------- SENSORS ----------------- //
    int get_rotor_encoder_value();
    long get_encoder_value();
    int  get_temperature();
    bool limit_switch_pressed(const int cs_limit_switch);
    bool is_valid_temperature(int temperature);
    bool is_at_top();

    // ---------- HOME ASSISTANT UPDATE ---------- //
    void update_rotor_encoder();
    void update_encoder();
    void update_temperature();

private:
       
    int grillIndex;
    GrillMQTT* mqtt;
    HardwareManager* hardware;
    ModeManager* modeManager;
    
    // Last values
    long lastEncoderValue;
    int lastRotorEncoderValue;
    int lastTemperatureValue;
};

#endif
