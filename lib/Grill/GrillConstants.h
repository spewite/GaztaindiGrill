#ifndef GRILL_CONSTANTS
#define GRILL_CONSTANTS

class GrillConstants {
public:

    static constexpr int NUM_GRILLS = 2;

    // Special values
    static constexpr int NO_TARGET = -999;
    static constexpr float ENCODER_ERROR = -9999.0f;
    
    // Hardware constants (PT100)
    static constexpr float RREF = 430.0f;
    static constexpr float RNOMINAL = 100.0f;
    
    // Margins
    static constexpr int POSITION_MARGIN = 2;        // For go_to position
    static constexpr int TEMPERATURE_MARGIN = 2;     // For go_to temperature
    static constexpr int ROTOR_MARGIN = 3;           // For go_to rotor
    
    // Timeouts
    static constexpr unsigned long RESET_TIMEOUT = 1000;     //  1 seconds
    static constexpr unsigned long MOVEMENT_TIMEOUT = 30000; // 30 segundos
    
    // Time intervals
    static constexpr unsigned long SENSOR_UPDATE_INTERVAL = 1500;
    static constexpr unsigned long PROGRAM_UPDATE_INTERVAL = 100;
    
    // System limits
    static constexpr int MAX_PROGRAM_STEPS = 50;

    // JSON field names for program steps
    static constexpr const char* JSON_TIME = "time";
    static constexpr const char* JSON_TEMPERATURE = "temperature";
    static constexpr const char* JSON_POSITION = "position";
    static constexpr const char* JSON_ROTATION = "rotation";
    static constexpr const char* JSON_ACTION = "action";
    
    // Sensor update topics
    static constexpr const char* TOPIC_UPDATE_POSITION = "update_position";
    static constexpr const char* TOPIC_UPDATE_TEMPERATURE = "update_temperature";
    static constexpr const char* TOPIC_UPDATE_TILT = "update_tilt";

    // Movement topics
    static constexpr const char* TOPIC_MOVE = "move";
    static constexpr const char* TOPIC_TILT = "tilt";

    // Set position topics
    static constexpr const char* TOPIC_SET_POSITION = "set_position";
    static constexpr const char* TOPIC_SET_TILT = "set_tilt";

    // Program topics
    static constexpr const char* TOPIC_EXECUTE_PROGRAM = "execute_program";
    static constexpr const char* TOPIC_CANCEL_PROGRAM = "cancel_program";
    
    // Mode topic
    static constexpr const char* TOPIC_SET_MODE = "set_mode";
    
    // Miscellaneous topics
    static constexpr const char* TOPIC_LOG = "log";
    static constexpr const char* TOPIC_RESTART = "restart";

    // Command payloads
    static constexpr const char* PAYLOAD_UP = "up";
    static constexpr const char* PAYLOAD_DOWN = "down";
    static constexpr const char* PAYLOAD_STOP = "stop";

    static constexpr const char* PAYLOAD_CLOCKWISE = "clockwise";
    static constexpr const char* PAYLOAD_COUNTER_CLOCKWISE = "counter_clockwise";

    static constexpr const char* PAYLOAD_SINGLE = "normal";
    static constexpr const char* PAYLOAD_DUAL = "dual";

    // Execution topics
    static constexpr const char* TOPIC_GET_PROGRAM_STATUS = "get_program_status";
    static constexpr const char* TOPIC_PROGRAM_STATUS_RESPONSE = "program_status_response";
    static constexpr const char* TOPIC_PROGRAM_STEP_CHANGED = "program_step_changed";

};

#endif
