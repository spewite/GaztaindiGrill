#ifndef GRILL_PROGRAM_H
#define GRILL_PROGRAM_H

#include <ArduinoJson.h>
#include <GrillMQTT.h>
#include <MovementManager.h>
#include <Utils.h>
#include <StatusLED.h>

struct Step {
    int time;
    int temperature;
    int position;
    int rotation;
    String action;
};

struct Program {
    int id = -1;
    String name;
    String description;
    String creatorName;
    int usageCount = 0;
    int stepsCount = 0;
    Step steps[GrillConstants::MAX_PROGRAM_STEPS];
};

class ProgramManager {
public:
    ProgramManager(int index, GrillMQTT* mqtt, MovementManager* movement, StatusLED* statusLed);

    // ----------------- PROGRAMS ----------------- //
    void update_program();
    void execute_program(const char* program);
    void publish_program_status();
    void finish_program(bool forcedCancelation);
    bool is_program_running();

private:

    int grillIndex;

    GrillMQTT* mqtt;
    MovementManager* movement;
    StatusLED* statusLed;

    // Por ahora solo se use el IDLE. Se deberia quitar esto? 
    // Quizas en un futuro se quiere mostar al usuario el estado del programa? 
    enum ProgramState {
        PROGRAM_IDLE,
        PROGRAM_RUNNING,
        PROGRAM_COMPLETED,
        PROGRAM_CANCELLED
    } programState = PROGRAM_IDLE;
    
    enum StepState {
        STEP_STARTING,
        STEP_MOVING_TO_TARGET,
        STEP_WAITING_TIME,
        STEP_EXECUTING_ACTION,
        STEP_COMPLETED
    } stepState = STEP_STARTING;

    void start_current_step();
    void check_target_reached();
    void check_time_elapsed();
    void execute_current_action();
    void advance_to_next_step();
    
    Program currentProgram;

    int programCurrentStep;
    unsigned long stepDurationStart;
    uint32_t stepStartUnix; // Timestamp of when a step started (this is to indicate in the client how many seconds has being executing)
};

#endif
