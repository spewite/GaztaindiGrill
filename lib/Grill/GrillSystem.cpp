#include <GrillSystem.h>
#include <Arduino.h>

GrillSystem::GrillSystem() : mqtt(nullptr), dualCoordinator(nullptr), modeManager(nullptr), previousMillisTemp(0) {
    for (int i = 0; i < GrillConstants::NUM_GRILLS; ++i) {
        grills[i] = nullptr;
    }
}

GrillSystem::~GrillSystem() {
    delete mqtt;
    delete dualCoordinator;
    delete modeManager;
    for (int i = 0; i < GrillConstants::NUM_GRILLS; ++i) {
        delete grills[i];
    }
}

bool GrillSystem::initialize_system(StatusLED* statusLed) {

    // Create the mode manager instance
    modeManager = new ModeManager();

    // Create system-level MQTT instance
    mqtt = new GrillMQTT(-1);
    mqtt->subscribe_to_system_topics();

    // Grill instantiation & start
    for (int i = 0; i < GrillConstants::NUM_GRILLS; ++i) {
        grills[i] = new Grill(i, modeManager, statusLed);
        if (grills[i]->setup_devices()) {
            Serial.println("The grill " + String(i) + " has been configured correctly");
            grills[i]->reset_system();
            grills[i]->subscribe_to_topics();
        } else {
            Serial.println("An error has occurred while configuring the devices of grill " + String(i));
            return false;
        }
    }
    
    // Initialize dual mode coordinator
    if (GrillConstants::NUM_GRILLS >= 2) {
        dualCoordinator = new DualModeCoordinator(grills[0], grills[1]);
    }
    
    return true;
}

void GrillSystem::update() {

    if (modeManager->requestedMode != modeManager->mode) {
        set_system_mode(modeManager->requestedMode);
    }
    
    // Handle dual mode
    handle_dual_mode();
    
    // Handle rotor operations
    handle_rotor_operations();
    
    // Update individual grills
    update_individual_grills();
    
    // Handle temperature updates
    handle_temperature_updates();
}

Grill* GrillSystem::get_grill(int index) {
    if (index >= 0 && index < GrillConstants::NUM_GRILLS) {
        return grills[index];
    }
    return nullptr;
}

bool GrillSystem::is_dual_mode_active() {
    return dualCoordinator && dualCoordinator->is_dual_mode_active();
}

void GrillSystem::handle_dual_mode() {
    if (dualCoordinator) {
        dualCoordinator->update();
    }
}

void GrillSystem::update_individual_grills() {
    
    for (int i = 0; i < GrillConstants::NUM_GRILLS; ++i) {
        if (grills[i]) {

            // Handle the stop
            grills[i]->handle_position_stop();
           
            // Handle program steps
            grills[i]->update_program();    
        
            // Update Home Assistant states
            grills[i]->update_encoder(); 
        }
    }
}

void GrillSystem::handle_rotor_operations() {
    
    // Only the left grill has a rotor
    if (grills[0]) {
        grills[0]->handle_rotor_stop();
        grills[0]->update_rotor_encoder();
    }
}

void GrillSystem::handle_temperature_updates() {
    // Temperature handling (currently commented out in original)
    // Uncomment and modify as needed:
    
    // if (grills[0]) {
    //     grills[0]->handle_temperature_stop(); 
    //     
    //     // Temperatura irakutzeko pausa, MQTT ez kargatzeko.
    //     unsigned long currentMillisTemp = millis();
    //     if (currentMillisTemp - previousMillisTemp >= intervalTemp) {
    //         previousMillisTemp = currentMillisTemp;
    //         grills[0]->update_temperature(); // Kontuan euki ezkerreko parrillak bakarrik eukikoula pt100
    //     }
    // }
}

void GrillSystem::set_system_mode(Mode newMode) {
    
    // If a program is being executed, do not allow mode change
    for (int i = 0; i < GrillConstants::NUM_GRILLS; ++i) {
        if (grills[i] && grills[i]->is_program_running()) {
            
            // Cancel the request by reverting to current mode
            mqtt->print("Program in progress. Mode change denied."); 

            modeManager->requestedMode = modeManager->mode;
            return;
        }
    }

    // If we reach here, it is safe to change mode
    for (int i = 0; i < GrillConstants::NUM_GRILLS; ++i) {
        if (grills[i]) {
            grills[i]->stop_lineal_actuator_raw();
        }
    }

    modeManager->confirmMode();
    Serial.println(newMode == DUAL ? "SYSTEM: DUAL MODE ACTIVATED" : "SYSTEM: SINGLE MODE ACTIVATED");
    
    // Notify the current mode back to the user
    String currentMode = modeManager->getCurrentMode();
    mqtt->publish_message(GrillConstants::TOPIC_CURRENT_MODE, currentMode, true); // Retained to ensure new clients get it
}

void GrillSystem::handle_mqtt_message(const char* pTopic, const char* pPayload) {

    
    String topic(pTopic);
    String payload(pPayload);

    mqtt->print("[GrillSystem::handle_mqtt_message] topic: " + topic);

    if (topic == GrillConstants::TOPIC_CURRENT_MODE) {
        mqtt->print("Syncing mode from MQTT...");
        if (payload == GrillConstants::PAYLOAD_SINGLE && modeManager->mode != SINGLE) {
            modeManager->requestMode(SINGLE);
        } else if (payload == GrillConstants::PAYLOAD_DUAL && modeManager->mode != DUAL) {
            modeManager->requestMode(DUAL);
        }
    }

    if (topic == GrillConstants::TOPIC_CMD_SYS_RESTART) {
        mqtt->print("Restarting entire system...");
        
        // Notify that the system is starting to reset
        mqtt->publish_message(GrillConstants::TOPIC_RESET_STATUS, GrillConstants::PAYLOAD_RESETTING, true);
        
        // Give some time for the MQTT message to be sent
        delay(500);
        
        // Software restart
        ESP.restart();
    }
    
    if (topic == GrillConstants::TOPIC_REQ_MODE_CHANGE)
    {
        mqtt->print("Received mode change request..."); 

        if (payload == GrillConstants::PAYLOAD_SINGLE)
        {
            modeManager->requestMode(SINGLE);
        }
        
        if (payload == GrillConstants::PAYLOAD_DUAL)
        {
            modeManager->requestMode(DUAL);
        }
    }

    if (topic == GrillConstants::TOPIC_CMD_REQ_CURRENT_MODE)
    {
        mqtt->print("Received current mode publish request..."); 
        String currentMode = modeManager->getCurrentMode();
        mqtt->publish_message(GrillConstants::TOPIC_CURRENT_MODE, currentMode, false);
    }
}


