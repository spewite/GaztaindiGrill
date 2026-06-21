#ifndef STATUS_LED_H
#define STATUS_LED_H

#include <FastLED.h>
#include <GRILL_config.h>

#define NUM_STATUS_LEDS 1 

// Defines the possible states of the status LED.
enum class LedState {
    OFF,
    CONNECTING_INTERNET,
    CONNECTING_MQTT,
    PULSING,
    RESETING,
    PROGRAM_RUNNING,
    ERROR
};

class StatusLED {
public:
    StatusLED();
    void begin();
    void setState(LedState newState);
    LedState getCurrentState();
    void update();

    // onTimeMs: The duration of the 'on' state in milliseconds.
    // offTimeMs: The duration of the 'off' state in milliseconds.
    // finalState: The state the LED will be in after the sequence ends.
    void pulse(uint8_t count, const CRGB& color, int onTimeMs, int offTimeMs, LedState finalState);

    // Shows a single, momentary white flash. Used for user action feedback.
    // This has priority over the current LED state.
    void show_action_pulse();

private:
    // --- LED Hardware & State ---
    CRGB leds[NUM_STATUS_LEDS]; 
    LedState currentState;
    LedState stateBeforeActionPulse; // Remembers the state before an action pulse.
    unsigned long lastBlinkTime;
    bool blinkState; // Used for simple, continuous blinking.

    // --- Action Pulse Logic ---
    bool isActionPulseActive;
    unsigned long actionPulseStartTime;

    // --- Custom Pulse Sequence Logic ---
    uint8_t pulseCounter;
    uint8_t pulseTotalCount;
    CRGB pulseColor;
    int pulseOnTime;
    int pulseOffTime;
    LedState pulseFinalState;
    
    // --- Private Helper Functions ---
    void blink(const CRGB& color, int interval);
    void handle_pulsing_state();
};

#endif
