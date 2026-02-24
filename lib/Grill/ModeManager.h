

#ifndef GRILL_MODE_H
#define GRILL_MODE_H

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
    
private:

    int grillIndex;
    
};
#endif
