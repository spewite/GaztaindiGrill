

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

    void requestMode(Mode newMode) {
        requestedMode = newMode;
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
