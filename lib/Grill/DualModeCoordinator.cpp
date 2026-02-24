#include "DualModeCoordinator.h"
#include "Grill.h"

DualModeCoordinator::DualModeCoordinator(Grill* grill0, Grill* grill1) 
    : grill0(grill0), grill1(grill1), resetState(IDLE) {
}

bool DualModeCoordinator::are_both_at_top() {
    return grill0->is_at_top() && grill1->is_at_top();
}

bool DualModeCoordinator::is_dual_mode_active() {
    // Dual mode will only be active if it has been reset
    return grill0->get_mode() == DUAL && resetState == READY;
}

void DualModeCoordinator::start_reset_sequence() {
    if (resetState == IDLE) {
        resetState = RESETTING;
        grill0->go_up_raw();
        grill1->go_up_raw();
    }
}

void DualModeCoordinator::update() {
    
    // If neither grill is in DUAL mode, reset the state and exit.
    if (grill0->get_mode() != DUAL && grill1->get_mode() != DUAL) {
        resetState = IDLE;
        return;
    }

    // --- State Machine ---
    switch (resetState) {
        case IDLE:
            // If DUAL mode has just been activated, initiate the reset
            if (grill0->get_mode() == DUAL || grill1->get_mode() == DUAL) {
                start_reset_sequence();
            }
            break;

        case RESETTING:

            // Notify the system is resetting
            client.publish(GrillConstants::TOPIC_RESET_STATUS, GrillConstants::PAYLOAD_RESETTING, true);

            // While resetting, check if both have reached the top.
            if (are_both_at_top()) {
                grill0->stop_lineal_actuator_raw();
                grill1->stop_lineal_actuator_raw();

                grill0->reset_encoder();
                grill1->reset_encoder();
                
                // Notify the sensors that this position is the "zero" for dual mode
                
                resetState = READY;
                
                // Notify the system has been resetted 
                client.publish(GrillConstants::TOPIC_RESET_STATUS, GrillConstants::PAYLOAD_RESET_READY, true);
            }
            break;

        case READY:
            // Once ready, execute normal synchronized movement.
            DualModeDirection direction = grill0->get_dual_direction();
            execute_synchronized_movement(direction);
            break;
    }
}

void DualModeCoordinator::execute_synchronized_movement(DualModeDirection direction) {
    // This functions is only executed when resetState is READY
    switch (direction) {
        case UPWARDS:
            grill0->go_up_raw();
            grill1->go_up_raw();
            break;
        case STILL:
            grill0->stop_lineal_actuator_raw();
            grill1->stop_lineal_actuator_raw();
            break;
        case DOWNWARDS:
            grill0->go_down_raw();
            grill1->go_down_raw();
            break;
    }
}