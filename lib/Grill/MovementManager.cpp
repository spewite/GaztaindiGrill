

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
    statusLed(statusLed),
    isLinearResetting(false)
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

void MovementManager::start_reset()
{
    statusLed->setState(LedState::RESETING);
    isLinearResetting = true;
    go_up(); 
    mqtt->print("Moving linear actuators to top");
}

bool MovementManager::check_reset_status()
{
    if (sensor->is_at_top()) {
        stop_lineal_actuator();
        hardware->reset_encoder(hardware->encoder);
        sensor->update_encoder();
        isLinearResetting = false;
        statusLed->pulse(3, CRGB::Green, 250, 250, LedState::OFF);
        return true;
    }
    return false;
}

void MovementManager::emergency_stop()
{
    stop_lineal_actuator();
    stop_rotor();
    isLinearResetting = false;
    targetTemperature = GrillConstants::NO_TARGET;
    targetDegrees = GrillConstants::NO_TARGET;
    targetPosition = GrillConstants::NO_TARGET;
    mqtt->print("EMERGENCY STOP EXECUTED");
}

bool MovementManager::is_resetting()
{
    return isLinearResetting;
}

bool MovementManager::has_any_active_target() {
    return (targetTemperature != GrillConstants::NO_TARGET ||
            targetPosition != GrillConstants::NO_TARGET ||
            targetDegrees != GrillConstants::NO_TARGET);    
}