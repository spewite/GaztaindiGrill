#include "StatusLED.h"

// Constructor: Initializes all state variables.
StatusLED::StatusLED()
    : currentState(LedState::OFF),
      stateBeforeActionPulse(LedState::OFF),
      lastBlinkTime(0),
      blinkState(false),
      isActionPulseActive(false),
      actionPulseStartTime(0),
      pulseCounter(0),
      pulseTotalCount(0),
      pulseColor(CRGB::Black),
      pulseOnTime(0),
      pulseOffTime(0),
      pulseFinalState(LedState::OFF) {
}

// Initializes the FastLED library and sets the initial state to OFF.
void StatusLED::begin() {
    FastLED.addLeds<WS2811, PIN_STATUS_LED, BRG>(leds, NUM_STATUS_LEDS);
    FastLED.setBrightness(50); // Adjust brightness as needed.
    setState(LedState::OFF);
}

// Sets a new state for the LED, resetting timers and counters if necessary.
void StatusLED::setState(LedState newState) {
    if (currentState == newState) return; // Avoid redundant state changes.
    
    currentState = newState;
    lastBlinkTime = 0;
    blinkState = false;
    
    // If the new state is not PULSING, reset the pulse sequence counter.
    if (newState != LedState::PULSING) {
        pulseCounter = 0;
        pulseTotalCount = 0;
    }

    update(); // Immediately update the LED to reflect the new state.
}

// Returns the current state of the LED.
LedState StatusLED::getCurrentState() {
    return currentState;
}

// Configures and starts a custom blinking sequence.
void StatusLED::pulse(uint8_t count, const CRGB& color, int onTimeMs, int offTimeMs, LedState finalState) {
    pulseTotalCount = count * 2; // Each pulse has an ON and OFF phase.
    pulseCounter = 0;
    pulseColor = color;
    pulseOnTime = onTimeMs;
    pulseOffTime = offTimeMs;
    pulseFinalState = finalState;
    setState(LedState::PULSING);
}

// Activates the momentary white flash for user feedback.
void StatusLED::show_action_pulse() {
    if (isActionPulseActive) return; // Prevent re-triggering while already active.

    isActionPulseActive = true;
    actionPulseStartTime = millis();
    stateBeforeActionPulse = currentState; // Save the current state to restore it later.
}

// This is the main loop function, which must be called repeatedly.
void StatusLED::update() {

    unsigned long currentTime = millis();

    // Priority 1: Handle the action pulse (momentary white flash).
    // This overrides any other state.
    if (isActionPulseActive) {
        if (currentTime - actionPulseStartTime < 250) {
            fill_solid(leds, NUM_STATUS_LEDS, CRGB::White);
        } else {
            isActionPulseActive = false; // The pulse has finished.
            // Restore the previous state. This will also trigger an immediate update.
            setState(stateBeforeActionPulse); 
        }
        FastLED.show();
        return; // Exit early to prevent the normal state machine from running.
    }

    // Serial.print("Estado actual del LED (numérico): ");
    // Serial.println(static_cast<int>(currentState));
    
    // Priority 2: Main state machine.
    switch (currentState) {
        case LedState::OFF:
            fill_solid(leds, NUM_STATUS_LEDS, CRGB::Black);
            break;
        case LedState::CONNECTING_INTERNET:
            blink(CRGB::Orange, 300);
            break;
        case LedState::CONNECTING_MQTT:
            blink(CRGB::Yellow, 200);
            break;
        case LedState::PULSING:
            handle_pulsing_state();
            break;
        case LedState::ERROR:
            blink(CRGB::Red, 150);
            break;
        case LedState::RESETING:
            blink(CRGB::Teal, 250);
            break;
        case LedState::PROGRAM_RUNNING:
            blink(CRGB::Purple, 1000);
            break;
    }
    FastLED.show();
}

// Handles the logic for a continuous, simple ON/OFF blink.
void StatusLED::blink(const CRGB& color, int interval) {
    unsigned long currentTime = millis();
    if (currentTime - lastBlinkTime >= interval) {
        lastBlinkTime = currentTime;
        blinkState = !blinkState; // Toggle state.
        
        if (blinkState) {
            fill_solid(leds, NUM_STATUS_LEDS, color);
        } else {
            fill_solid(leds, NUM_STATUS_LEDS, CRGB::Black);
        }
    }
}

void StatusLED::handle_pulsing_state() {

    // Check if the pulse sequence has completed.
    if (pulseCounter >= pulseTotalCount) {
        setState(pulseFinalState); // Set the final state when done.
        return;
    }
    
    unsigned long currentTime = millis();
    // Determine the interval for the current phase (ON or OFF).
    int currentInterval = blinkState ? pulseOnTime : pulseOffTime;

    if (currentTime - lastBlinkTime >= currentInterval) {
        lastBlinkTime = currentTime;
        blinkState = !blinkState; // Toggle between ON and OFF.
        
        if (blinkState) {
            fill_solid(leds, NUM_STATUS_LEDS, pulseColor);
        } else {
            fill_solid(leds, NUM_STATUS_LEDS, CRGB::Black);
        }
        pulseCounter++; // Increment the counter for each phase change.
    }
}
