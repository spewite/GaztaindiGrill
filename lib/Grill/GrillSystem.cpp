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

    // Setup all grills and start reset simultaneously
    for (int i = 0; i < GrillConstants::NUM_GRILLS; ++i) {
        grills[i] = new Grill(i, modeManager, statusLed);
        if (grills[i]->setup_devices()) {
            Serial.println("The grill " + String(i) + " has been configured correctly. Starting reset...");
            statusLed->setState(LedState::RESETING);
            grills[i]->start_reset(); // Start moving up without blocking
        } else {
            Serial.println("An error has occurred while configuring the devices of grill " + String(i));
            return false;
        }
    }

    // Now that all objects are created, subscribe to everything at once
    resubscribe_all();

    // Wait for all grills to complete reset (non-blocking wait)
    Serial.println("Waiting for all grills to reach top position...");
    bool allResetted = false;
    while (!allResetted) {
        allResetted = true;
        for (int i = 0; i < GrillConstants::NUM_GRILLS; ++i) {
            if (grills[i] && !grills[i]->check_reset_status()) {
                allResetted = false; // At least one grill is still moving
            }
        }

        // Process MQTT messages (like emergency stop) and update LEDs
        client.loop();
        statusLed->update();
        delay(10);
    }

    Serial.println("All grills resetted successfully.");
    statusLed->pulse(3, CRGB::Green, 250, 250, LedState::OFF);
    
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

            // Deferred answer: the handler that received this request returned long ago, so the
            // requester is whoever ModeManager recorded, not whatever message arrived last.
            mqtt->reply_to(modeManager->requestedByRequestId,
                           modeManager->requestedByCommand,
                           false,
                           GrillConstants::ERROR_MODE_CHANGE_DENIED);

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

    // Deferred answer, same as the rejection above.
    mqtt->reply_to(modeManager->requestedByRequestId,
                   modeManager->requestedByCommand,
                   true,
                   nullptr);

    // Notify the current mode back to the user
    String currentMode = modeManager->getCurrentMode();
    mqtt->publish_message(GrillConstants::TOPIC_CURRENT_MODE, currentMode, true); // Retained to ensure new clients get it
}

void GrillSystem::reply_ok_if_unanswered(GrillRequest& request) {
    // See the note in Grill::reply_ok_if_unanswered(): EVERYONE means nobody asked, and the
    // ESP32 subscribes to its own retained reset_status/current_mode topics, so this would
    // otherwise fire on every boot.
    if (request.replied) { return; }
    if (request.id == GrillConstants::PAYLOAD_REQUEST_ID_EVERYONE) { return; }
    mqtt->reply_ok(request);
}

void GrillSystem::handle_mqtt_message(const char* pTopic, GrillRequest& request) {


    String topic(pTopic);
    String payload = request.value;

    mqtt->print("[GrillSystem::handle_mqtt_message] topic: " + topic);

    if (topic == GrillConstants::TOPIC_CURRENT_MODE) {
        mqtt->print("Syncing mode from MQTT...");
        // Retained state the ESP32 reads back at boot, not a user command: nobody is waiting
        // for an answer, so the outcome is broadcast rather than addressed.
        if (payload == GrillConstants::PAYLOAD_SINGLE && modeManager->mode != SINGLE) {
            modeManager->requestMode(SINGLE, GrillConstants::PAYLOAD_REQUEST_ID_EVERYONE, topic);
        } else if (payload == GrillConstants::PAYLOAD_DUAL && modeManager->mode != DUAL) {
            modeManager->requestMode(DUAL, GrillConstants::PAYLOAD_REQUEST_ID_EVERYONE, topic);
        }
        mqtt->defer(request);
    }

    if (topic == GrillConstants::TOPIC_CMD_SYS_RESTART) {
        mqtt->print("Restarting entire system...");

        // Answered here rather than by the dispatcher: ESP.restart() below never returns, so
        // an automatic reply would never be reached.
        mqtt->reply_ok(request);

        // Notify that the system is starting to reset
        mqtt->publish_message(GrillConstants::TOPIC_RESET_STATUS, GrillConstants::PAYLOAD_RESETTING, true);
        
        // Give some time for the MQTT message to be sent
        delay(500);
        
        // Software restart
        ESP.restart();
    }

    if (topic == GrillConstants::TOPIC_CMD_SYS_EMERGENCY_STOP) {
        mqtt->print("GLOBAL EMERGENCY STOP RECEIVED");
        for (int i = 0; i < GrillConstants::NUM_GRILLS; ++i) {
            if (grills[i]) {
                grills[i]->emergency_stop();
            }
        }
    }
    
    if (topic == GrillConstants::TOPIC_CMD_REQ_CURRENT_MODE)
    {
        mqtt->print("Received current mode publish request..."); 
        String currentMode = modeManager->getCurrentMode();
        mqtt->publish_message(GrillConstants::TOPIC_CURRENT_MODE, currentMode, false);
    }
    
    if (topic == GrillConstants::TOPIC_REQ_MODE_CHANGE)
    {
        mqtt->print("Received mode change request...");

        if (payload == GrillConstants::PAYLOAD_SINGLE)
        {
            modeManager->requestMode(SINGLE, request.id, request.command);
        }

        if (payload == GrillConstants::PAYLOAD_DUAL)
        {
            modeManager->requestMode(DUAL, request.id, request.command);
        }

        // Only the intent is recorded here. set_system_mode() decides in a later loop iteration
        // and answers then, so no automatic ok must go out now or it would contradict it.
        mqtt->defer(request);
    }


}

void GrillSystem::resubscribe_all() {
    if (mqtt) {
        mqtt->subscribe_to_system_topics();
    }
    for (int i = 0; i < GrillConstants::NUM_GRILLS; ++i) {
        if (grills[i]) {
            grills[i]->subscribe_to_topics();
        }
    }
}


