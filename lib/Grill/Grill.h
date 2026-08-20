#ifndef GRILL_H
#define GRILL_H

#include <Wire.h>
#include <SPI.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <SerialTelnet.h>

#include <GrillMQTT.h>
#include <HardwareManager.h>
#include <GrillSensor.h>
#include <ModeManager.h>
#include <MovementManager.h>
#include <ProgramManager.h>
#include <ModeManager.h>
#include <StatusLed.h>

extern PubSubClient client;

class Grill {
public:

    Grill(int index, ModeManager* sharedModeManager, StatusLED* statusLed);

    // Setup
    bool setup_devices();
    void reset_encoder();

    void start_reset();
    bool check_reset_status();
    bool check_rotor_reset_status();
    void emergency_stop();
    bool is_resetting();
    
    // Movement
    void go_up_raw();
    void go_down_raw();
    void stop_lineal_actuator_raw();
    void go_to(int position);
    
    // Sensors
    int get_temperature();
    long get_encoder();
    bool is_at_top();

    // Sensor updates
    void update_rotor_encoder();
    void update_encoder(); 

    // Handle stops
    void handle_rotor_stop();
    void handle_position_stop();
    void handle_temperature_stop();

    // Mode
    DualModeDirection get_dual_direction();
    Mode get_mode();

    // Programs
    void update_program();
    bool is_program_running();
    
    // MQTT
    void subscribe_to_topics();
    void handle_mqtt_message(const char* pAction, GrillRequest& request);

    // Called by the dispatcher once the handler returns: anything that did not reject succeeded.
    void reply_ok_if_unanswered(GrillRequest& request);



private:
    
    int index;
    StatusLED* statusLed;
    GrillMQTT* mqtt;
    HardwareManager* hardware;
    GrillSensor* sensor;
    ModeManager* modeManager;
    MovementManager* movement;
    ProgramManager* programManager;
    
};

#endif
