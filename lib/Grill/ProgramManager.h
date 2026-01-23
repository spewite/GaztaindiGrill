#ifndef GRILL_PROGRAM_H
#define GRILL_PROGRAM_H

#include <ArduinoJson.h>
#include <GrillMQTT.h>
#include <MovementManager.h>
#include <StatusLED.h>

class ProgramManager {
public:
    ProgramManager(int index, GrillMQTT* mqtt, MovementManager* movement, StatusLED* statusLed);

    // ----------------- PROGRAMS ----------------- //
    void cancel_program();
    void update_program();
    void execute_program(const char* program);
    void publish_program_status();
private:

    int grillIndex;

    GrillMQTT* mqtt;
    MovementManager* movement;
    StatusLED* statusLed;

    enum ProgramState {
        PROGRAM_IDLE,
        PROGRAM_RUNNING,
        PROGRAM_COMPLETED,
        PROGRAM_CANCELLED,
        PROGRAM_ERROR
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
    
    struct Step {
        int time;
        int temperature;
        int position;
        int rotation;
        const char* action;
    };
    Step steps[GrillConstants::MAX_PROGRAM_STEPS];
    
    int programId;
    int programStepsCount;
    int programCurrentStep;
    unsigned long stepDurationStart;
};

#endif
