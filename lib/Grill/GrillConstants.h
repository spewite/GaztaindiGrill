#ifndef GRILL_CONSTANTS_H
#define GRILL_CONSTANTS_H

class GrillConstants {
public:

    // Maximum ammount of steps permitted in a program
    static constexpr int MAX_PROGRAM_STEPS = 50;

    static constexpr int NUM_GRILLS = 2;

    // Special values
    static constexpr int NO_TARGET = -999;
    static constexpr float ENCODER_ERROR = -9999.0f;
    
    // Hardware constants (PT100)
    static constexpr float RREF = 430.0f;
    static constexpr float RNOMINAL = 100.0f;
    
    // Margins
    static constexpr int POSITION_MARGIN = 0;
    static constexpr int TEMPERATURE_MARGIN = 2;
    static constexpr int ROTOR_MARGIN = 3;
    static constexpr int SYNC_MARGIN = 0; // Margin for dual mode synchronization
    
    // Timeouts
    static constexpr unsigned long RESET_TIMEOUT = 1000;
    static constexpr unsigned long MOVEMENT_TIMEOUT = 30000;
    
    // Time intervals
    static constexpr unsigned long SENSOR_UPDATE_INTERVAL = 1500;
    static constexpr unsigned long PROGRAM_UPDATE_INTERVAL = 100;
    
    // --- MQTT TOPICS ---
    
    // -- Global Topics --
    static constexpr const char* TOPIC_LWT = "grill/connection";
    static constexpr const char* TOPIC_RESET_STATUS = "grill/reset_status";
    static constexpr const char* TOPIC_CURRENT_MODE = "grill/current_mode";
    static constexpr const char* TOPIC_REQ_MODE_CHANGE = "grill/request_mode_change";
    static constexpr const char* TOPIC_CMD_REQ_CURRENT_MODE = "grill/request_current_mode";
    static constexpr const char* TOPIC_CMD_SYS_RESTART = "grill/restart";
    static constexpr const char* TOPIC_CMD_SYS_EMERGENCY_STOP = "grill/emergency_stop";
    static constexpr const char* TOPIC_MQTT_LOGGER = "grill/mqtt_logger";
    static constexpr const char* TOPIC_TIME = "grill/time"; // current UTC unix time (retained)
    
    // -- Command Topics (Client -> ESP32) --
    // Base: grill/{id}/...
    static constexpr const char* TOPIC_CMD_MOVE_VERTICAL = "action/movement/vertical";
    static constexpr const char* TOPIC_CMD_MOVE_ROTATION = "action/movement/rotation";
    static constexpr const char* TOPIC_CMD_SET_POSITION = "action/movement/set_position";
    static constexpr const char* TOPIC_CMD_SET_ROTATION = "action/movement/set_rotation";
    
    static constexpr const char* TOPIC_CMD_PROG_EXECUTE = "action/program/execute";
    static constexpr const char* TOPIC_CMD_PROG_CANCEL = "action/program/cancel";
    static constexpr const char* TOPIC_CMD_REQ_PROG_STATUS = "action/request/program_status";   
    
    // -- State Topics (ESP32 -> Client) --
    // Base: grill/{id}/...
    static constexpr const char* TOPIC_LOG = "log";
    static constexpr const char* TOPIC_STATE_SENSOR_POSITION = "status/sensor/position";
    static constexpr const char* TOPIC_STATE_SENSOR_ROTATION = "status/sensor/rotation";
    static constexpr const char* TOPIC_STATE_SENSOR_TEMP = "status/sensor/temperature";

    static constexpr const char* TOPIC_STATE_PROG_CURRENT = "status/program/current";

    // --- MQTT PAYLOADS ---
    
    // JSON field names for program steps
    static constexpr const char* JSON_TIME = "time";
    static constexpr const char* JSON_TEMPERATURE = "temperature";
    static constexpr const char* JSON_POSITION = "position";
    static constexpr const char* JSON_ROTATION = "rotation";
    static constexpr const char* JSON_ACTION = "action";
    static constexpr const char* JSON_REFERENCE_TYPE = "referenceType";


    // General payloads
    static constexpr const char* PAYLOAD_LWT_ONLINE  = "online";
    static constexpr const char* PAYLOAD_LWT_OFFLINE = "offline";

    static constexpr const char* PAYLOAD_RESETTING = "resetting";
    static constexpr const char* PAYLOAD_RESET_READY = "ready";


    // Command payloads
    static constexpr const char* PAYLOAD_UP = "up";
    static constexpr const char* PAYLOAD_DOWN = "down";
    static constexpr const char* PAYLOAD_STOP = "stop";
    static constexpr const char* PAYLOAD_CLOCKWISE = "clockwise";
    static constexpr const char* PAYLOAD_COUNTER_CLOCKWISE = "counter_clockwise";
    static constexpr const char* PAYLOAD_SINGLE = "single";
    static constexpr const char* PAYLOAD_DUAL = "dual";

    // Program reference type payloads
    static constexpr const char* PAYLOAD_REFERENCE_TYPE_ABSOLUTE = "absolute";
    static constexpr const char* PAYLOAD_REFERENCE_TYPE_RELATIVE = "relative";
};

#endif
