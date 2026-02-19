#include <Arduino.h>
#include <GRILL_config.h>
#include <Grill.h>

extern PubSubClient client;

Grill::Grill(int index, ModeManager* sharedModeManager, StatusLED* statusLed) :
        index(index),
        hardware(nullptr),
        mqtt(nullptr),
        statusLed(statusLed),
        modeManager(sharedModeManager),
        sensor(nullptr),
        movement(nullptr),
        programManager(nullptr) 
    {

        mqtt = new GrillMQTT(index);
        hardware = new HardwareManager(index, mqtt);
        sensor = new GrillSensor(index, mqtt, hardware, modeManager);
        movement = new MovementManager(index, mqtt, hardware, sensor, modeManager, statusLed);
        programManager = new ProgramManager(index, mqtt, movement, statusLed);

    }

//
// Setup
//
bool Grill::setup_devices() {
    return hardware->setup_devices();
}


void Grill::reset_system() {
    movement->reset_system();
}

//
// Movements
//
void Grill::go_up() {
    movement->go_up();
}

void Grill::go_down() {
    movement->go_down();
}

void Grill::go_to(int position) {
    movement->go_to(position);
}

void Grill::stop_lineal_actuator() {
    movement->stop_lineal_actuator();
}

//
// Sensors
//
int Grill::get_temperature() {
    return sensor->get_temperature();
}

long Grill::get_encoder() {
    return sensor->get_encoder_value();
}

bool Grill::is_at_top() {
    return sensor->is_at_top();
}

bool Grill::is_at_top_dual() {
    return sensor->is_at_top_dual;
}

void Grill::set_is_at_top_dual(bool isAtTop) {
    sensor->is_at_top_dual = isAtTop;
}

//
// Sensor updates
//
void Grill::update_encoder() {
    sensor->update_encoder();
}

void Grill::update_rotor_encoder() {
    sensor->update_rotor_encoder();
}

//
// Handle stops
//
void Grill::handle_rotor_stop() {
    movement->handle_rotor_stop();
}

void Grill::handle_position_stop() {
    movement->handle_position_stop();
}

void Grill::handle_temperature_stop() {
    movement->handle_temperature_stop();
}

//
// Mode
//
DualModeDirection Grill::get_dual_direction() {
    return modeManager->dual_direction;
}

Mode Grill::get_mode() {
    return modeManager->mode;
}

//
// Programs
//
void Grill::execute_program(const char* program) {
    programManager->execute_program(program);
}

void Grill::update_program() {
    programManager->update_program();
}

//
// MQTT
//

void Grill::subscribe_to_topics() {
    mqtt->subscribe_to_topics();
}

void Grill::handle_mqtt_message(const char* pAction, const char* pPayload) {
    String topic(pAction);
    String payload(pPayload);

    // We don't log the log topic itself to avoid loops
    if (topic != GrillConstants::TOPIC_LOG) {
        mqtt->print("Action received: " + topic + " -> " + payload);

        if (statusLed && topic != GrillConstants::TOPIC_CMD_PROG_CANCEL) {
            statusLed->show_action_pulse();
        }
    }

    if (topic == GrillConstants::TOPIC_CMD_MOVE_VERTICAL) {
        if (payload == GrillConstants::PAYLOAD_UP) {
            movement->go_up();
        } else if (payload == GrillConstants::PAYLOAD_DOWN) {
            movement->go_down();
        } else if (payload == GrillConstants::PAYLOAD_STOP) {
            movement->stop_lineal_actuator();
        }
    }  

    if (topic == GrillConstants::TOPIC_CMD_MOVE_ROTATION) {
        if (payload == GrillConstants::PAYLOAD_CLOCKWISE) {
            movement->rotate_clockwise();
        } else if (payload == GrillConstants::PAYLOAD_COUNTER_CLOCKWISE) {
            movement->rotate_counter_clockwise();
        } else if (payload == GrillConstants::PAYLOAD_STOP) {
            movement->stop_rotor();
        }
    }  

    if (topic == GrillConstants::TOPIC_CMD_SET_POSITION) {
        int posicion = payload.toInt();
        movement->go_to(posicion);
    }
    
    if (topic == GrillConstants::TOPIC_CMD_SYS_RESTART) {
        mqtt->print("Restarting system...");
        // NOTE: Add ESP.restart() here if needed
    }
    
    if (topic == GrillConstants::TOPIC_CMD_PROG_EXECUTE) {
        mqtt->print("Executing a program..."); 
        programManager->execute_program(pPayload);
    }
    
    if (topic == GrillConstants::TOPIC_CMD_PROG_CANCEL) {
        programManager->finish_program(true);
        mqtt->print("Program cancelled");
    }
    
    if (topic == GrillConstants::TOPIC_CMD_SET_ROTATION)
    {
        int grades = payload.toInt();
        movement->go_to_rotor(grades);
    }
    
    if (topic == GrillConstants::TOPIC_CMD_SYS_SET_MODE)
    {
        if (payload == GrillConstants::PAYLOAD_SINGLE)
        {
            modeManager->mode = SINGLE;
            movement->stop_lineal_actuator();
            movement->stop_rotor();
        }
        
        if (payload == GrillConstants::PAYLOAD_DUAL)
        {
            modeManager->mode = DUAL;
        }
    }

    
    if (topic == GrillConstants::TOPIC_CMD_REQ_PROG_STATUS) {
        programManager->publish_program_status();
    }
}
