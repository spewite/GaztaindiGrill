#ifndef GRILL_MOVEMENT_H
#define GRILL_MOVEMENT_H

#include <GrillMQTT.h>
#include <HardwareManager.h>
#include <GrillSensor.h>
#include <ModeManager.h>
#include <GrillConstants.h>
#include <StatusLED.h>

class MovementManager {
public:
    MovementManager(int index, GrillMQTT* mqtt, HardwareManager* hardware, GrillSensor* sensor, ModeManager* modeManager, StatusLED* statusLed);

    // ---------------- MOVEMENTS ----------------- //
    void go_up();
    void go_up_raw();
    void go_down();
    void go_down_raw();
    void stop_lineal_actuator();
    void stop_lineal_actuator_raw();

    // Only grill 0 is built with a rotor; on grill 1 hardware->rotor is null.
    bool has_rotor();

    void turn_around();
    void rotate_clockwise();
    void rotate_counter_clockwise();
    void stop_rotor();

    // ------------------- GO_TO ------------------ //
    void go_to(int position);
    void go_to_temp(int temperature);
    void go_to_rotor(int grades);

    // ------------------- RESETS ------------------ //
    void start_reset();
    bool check_reset_status();
    void emergency_stop();
    bool is_resetting();

    // ---- HANDLE STOPS (GO_TO / PROGRAM) ---- //
    void handle_rotor_stop();
    void handle_position_stop();
    void handle_temperature_stop();
    
    // -------------- GO_TO TARGETS ------------- //
    int targetTemperature;
    int targetDegrees;
    int targetPosition; 
    bool has_any_active_target();

private:

    int grillIndex;
    bool isLinearResetting;

    StatusLED* statusLed;
    GrillMQTT* mqtt;
    HardwareManager* hardware;
    GrillSensor* sensor;
    ModeManager* modeManager;

};

#endif
