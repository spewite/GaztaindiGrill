#ifndef GRILL_SYSTEM_H
#define GRILL_SYSTEM_H

#include <GrillConfig.h>
#include <Grill.h>
#include <GrillMQTT.h>
#include <DualModeCoordinator.h>
#include <ModeManager.h> 

class GrillSystem {
public:
    GrillSystem();
    ~GrillSystem();
    
    // Setup of the system
    bool initialize_system(StatusLED* statusLed);
    
    // Main loop
    void update();
    
    // Access to individual grills
    Grill* get_grill(int index);

    // Change the mode (individual, dual)
    void set_system_mode(Mode newMode);
    
    // Dual mode management
    bool is_dual_mode_active();
    void handle_dual_mode();

    // MQTT Handling
    void handle_mqtt_message(const char* topic, GrillRequest& request);

    // Called by the dispatcher once the handler returns: anything that did not reject succeeded.
    void reply_ok_if_unanswered(GrillRequest& request);
    void resubscribe_all();
    
private:
    Grill* grills[GrillConstants::NUM_GRILLS];
    GrillMQTT* mqtt;
    DualModeCoordinator* dualCoordinator;
    ModeManager* modeManager;
    
    void update_individual_grills();
    void handle_rotor_operations();
    void handle_temperature_updates();
    
    // Temperature update timing
    unsigned long previousMillisTemp;
    const long intervalTemp = 1500;
};

#endif
