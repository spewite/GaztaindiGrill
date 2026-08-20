

#ifndef GRILL_MODE_H
#define GRILL_MODE_H

#include <Arduino.h>
#include <GrillConstants.h>

enum Mode {
    SINGLE,
    DUAL
};

enum DualModeDirection {
    UPWARDS,
    STILL,
    DOWNWARDS
};

class ModeManager {
public:

    DualModeDirection dual_direction = STILL;
    Mode mode = SINGLE;
    Mode requestedMode = SINGLE;

    // Who asked for the pending mode change. The request is only recorded here; the accept or
    // reject happens later in GrillSystem::update(), by which time the incoming message is long
    // gone, so the id has to travel with the pending operation to reach the right client.
    String requestedByRequestId = GrillConstants::PAYLOAD_REQUEST_ID_EVERYONE;
    String requestedByCommand = "";

    void requestMode(Mode newMode, const String& requestId, const String& command) {
        requestedMode = newMode;
        requestedByRequestId = requestId;
        requestedByCommand = command;
    }

    void confirmMode() {
        mode = requestedMode;
        dual_direction = STILL;
    }

    String getCurrentMode() {
         return this->mode == SINGLE ? GrillConstants::PAYLOAD_SINGLE : GrillConstants::PAYLOAD_DUAL;
    }
    
private:

    int grillIndex;
    
};
#endif
