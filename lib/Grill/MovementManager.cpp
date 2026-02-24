

#include <MovementManager.h>

MovementManager::MovementManager(int index, GrillMQTT* mqtt, HardwareManager* hardware, GrillSensor* sensor, ModeManager* modeManager, StatusLED* statusLed): 
    grillIndex(index), 
    mqtt(mqtt),
    hardware(hardware),
    sensor(sensor),
    modeManager(modeManager),
    targetPosition(GrillConstants::NO_TARGET),
    targetDegrees(GrillConstants::NO_TARGET),
    targetTemperature(GrillConstants::NO_TARGET),
    statusLed(statusLed)
     {}


/// ----------- LINEAL ACTUATOR ----------- ///

void MovementManager::go_up() {

    if (modeManager->mode == DUAL)
    {
        modeManager->dual_direction = UPWARDS;
    } else 
    {
        go_up_raw();
    }
}

void MovementManager::go_up_raw() {
    hardware->drive->setSpeed(-255);
}

void MovementManager::go_down() {
    if (modeManager->mode == DUAL)
    {
        modeManager->dual_direction = DOWNWARDS;
    } else 
    {
        go_down_raw();
    }
}

void MovementManager::go_down_raw() {
    hardware->drive->setSpeed(255);
}

void MovementManager::stop_lineal_actuator() {
    if (modeManager->mode == DUAL)
    {
        modeManager->dual_direction = STILL;
    } else 
    {
        stop_lineal_actuator_raw();
    }
}

void MovementManager::stop_lineal_actuator_raw() {
    hardware->drive->setSpeed(0);
}

/// ----------- ROTOR ----------- ///

void MovementManager::rotate_clockwise()
{
    hardware->rotor->rotate_clockwise();
}

void MovementManager::rotate_counter_clockwise()
{
    hardware->rotor->rotate_counter_clockwise();
}

void MovementManager::stop_rotor()
{
    hardware->rotor->stop();
}

void MovementManager::turn_around() {
    mqtt->print("Turning around");
    int currentInclination = sensor->get_rotor_encoder_value();
    int targetInclination = (currentInclination + 180) % 360;
    go_to_rotor(targetInclination);
}


/// -------------------------- ///
///            GO TOs          /// 
/// -------------------------- ///

// ------------- ROTOR ------------- //

void MovementManager::go_to_rotor(int degrees) {

    if (degrees < 0 || degrees >= 360) {
        mqtt->print("Rotor degrees out of range");
        return;
    }  
     
    int currentRotorPosition = sensor->get_rotor_encoder_value();
    targetDegrees = degrees;
     
    int differenceRight = (targetDegrees - currentRotorPosition + 360) % 360;
    int differenceLeft = (currentRotorPosition - targetDegrees + 360) % 360;
     
    mqtt->print("New target: " + String(targetDegrees) + " (current: " + String(currentRotorPosition) + ")");

    // In the handle_rotor_stop() function that is called in loop, we handle when we have to stop
    if (differenceRight < differenceLeft) 
    {
        rotate_counter_clockwise();
    } else 
    {
        rotate_clockwise();
    }
}

void MovementManager::handle_rotor_stop() {
    
    int currentRotorPosition = sensor->get_rotor_encoder_value();

    if (abs(currentRotorPosition-targetDegrees) <= GrillConstants::ROTOR_MARGIN && targetDegrees != GrillConstants::NO_TARGET ) { 
        stop_rotor();
        targetDegrees = GrillConstants::NO_TARGET;
    } 
}

// ------------- LINEAL ACTUATOR ------------- //

void MovementManager::go_to(int position) {

    if (position < 0) position = 0;
    if (position > 100) position = 99;
    
    targetPosition = position;
    int currentPercentage = sensor->get_encoder_value();

    mqtt->print("New target: " + String(position) + " (current: " + String(currentPercentage) + ")");

    // In the function handle_temperature_stop(), which is called in loop, we handle when we have to stop.
    if (currentPercentage < position) {
        go_up();
    } else if (currentPercentage > position) {
        go_down();
    }
}

void MovementManager::handle_position_stop() {
    int currentPercentage = sensor->get_encoder_value();

    if (abs(currentPercentage - targetPosition) <= GrillConstants::POSITION_MARGIN && targetPosition != GrillConstants::NO_TARGET ) {
        stop_lineal_actuator();
        targetPosition = GrillConstants::NO_TARGET;
    } 
}


void MovementManager::go_to_temp(int temperature) {

    targetTemperature = temperature;
    int currentTemperature = sensor->get_temperature();

    // If the temperature is not valid, we exit the method
    if (!sensor->is_valid_temperature(currentTemperature)) {return;}

    mqtt->print("New target: " + String(targetTemperature) + " (current: " + String(currentTemperature) + ")");

    // In the function handle_temperature_stop(), which is called in loop, we handle when we have to stop.
    if (currentTemperature < targetTemperature) {
        go_up();
    } else if (currentTemperature > targetTemperature) {
        go_down();
    }
}

void MovementManager::handle_temperature_stop() {

    int currentTemperature = sensor->get_temperature();
 
    // If the temperature is not valid, we exit the method
    if (!sensor->is_valid_temperature(currentTemperature)) {return;}

    if (abs(currentTemperature - targetTemperature) <= GrillConstants::TEMPERATURE_MARGIN && targetTemperature != GrillConstants::NO_TARGET ) {
        stop_lineal_actuator();
        targetTemperature = GrillConstants::NO_TARGET;
    } 
}

/// ------------------------------------ ///
///             RESET SYSTEMS            /// 
/// ------------------------------------ ///

void MovementManager::reset_system() {
    
    // Start led indicator
    statusLed->setState(LedState::RESETING);

    mqtt->print("Resetting devices for grill " + String(grillIndex));   

    // ------------- RESET ROTOR ------------- //
    if (index == 0)
    {
        // reset_rotor(); 
    }

    // ------------- RESET LINEAL ACTUATOR ------------- //
    reset_linear_actuators();
    
    // ------------- RESET ENCODER ------------- //
    hardware->reset_encoder(hardware->encoder);
    sensor->update_encoder();

    mqtt->print("Devices reset");  
    
    // Stop led indicator
    statusLed->pulse(3, CRGB::Green, 250, 250, LedState::OFF);
    
}

void MovementManager::reset_rotor()
{
    mqtt->print("Resetting rotor");
    rotate_clockwise();

    // To avoid printing the message all the time, make a non-blocking delay so it can receive MQTT
    unsigned long previousMessageMillis = 0;
    
    while (!sensor->limit_switch_pressed(PIN_CS_LIMIT_ROTOR)) {
        unsigned long currentMillis = millis();
        if (currentMillis - previousMessageMillis >= GrillConstants::RESET_TIMEOUT) {
            previousMessageMillis = currentMillis;
            mqtt->print("Resetting rotor...");
        }
        client.loop();
    }

    stop_rotor();
    hardware->reset_rotor_encoder();
}

void MovementManager::reset_linear_actuators()
{
    go_up(); 
    mqtt->print("Moving linear actuators to top");

    // To avoid printing the message all the time, make a non-blocking delay so it can receive MQTT
    unsigned long previousMessageMillis = 0;

    while (!sensor->is_at_top()) {
        
        unsigned long currentMillis = millis();
        if (currentMillis - previousMessageMillis >= GrillConstants::RESET_TIMEOUT) {
            previousMessageMillis = currentMillis;
            mqtt->print("Moving linear actuators to top...");
        }
        client.loop();
        statusLed->update();
    }

    mqtt->print("Linear actuators at top");
    stop_lineal_actuator();
}

bool MovementManager::has_any_active_target() {
    return (targetTemperature != GrillConstants::NO_TARGET ||
            targetPosition != GrillConstants::NO_TARGET ||
            targetDegrees != GrillConstants::NO_TARGET);    
}