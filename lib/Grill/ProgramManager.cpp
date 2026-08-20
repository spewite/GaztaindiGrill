#include <ProgramManager.h>

ProgramManager::ProgramManager(int index, GrillMQTT* mqtt, MovementManager* movement, GrillSensor* sensor, StatusLED* statusLed) :
    grillIndex(index),
    mqtt(mqtt),
    movement(movement),
    sensor(sensor),
    statusLed(statusLed),
    programCurrentStep(0),
    stepDurationStart(0),
    stepStartUnix(0),
    positionAnchor(0)
     {}

    
void ProgramManager::execute_program(GrillRequest& request) {

    // Cancelar cualquier programa anterior.
    finish_program(true);

    // Usamos JsonDocument para que v7 gestione la memoria
    JsonDocument envelope;
    DeserializationError error = deserializeJson(envelope, request.raw);

    if (error) {
        mqtt->print("Error deserializing the program JSON.");
        mqtt->reply_error(request, GrillConstants::ERROR_INVALID_JSON);
        return;
    }

    // The program itself travels under "value", like every other command's payload.
    JsonObject doc = envelope[GrillConstants::JSON_VALUE];

    // Extract the program Id. We will use -1 if doesnt exist.
    currentProgram.id = doc["programId"] | -1;
    currentProgram.description = doc["description"] | "";
    currentProgram.name = doc["name"] | "";
    currentProgram.creatorName = doc["creatorName"] | "";
    currentProgram.usageCount = doc["usageCount"] | -1;
    currentProgram.referenceType = doc[GrillConstants::JSON_REFERENCE_TYPE] | GrillConstants::PAYLOAD_REFERENCE_TYPE_ABSOLUTE;

    // Anchor for "relative" referenceType: the grill's position right now, before the program moves it.
    // Uses the filtered last-known reading (not a raw get_encoder_value() read), since a single transient
    // sensor glitch here would silently poison the target for the whole program (no retry happens later).
    positionAnchor = sensor->get_last_known_position();

    mqtt->print("Executing program with ID: " + String(currentProgram.id) +
                " | referenceType: " + currentProgram.referenceType +
                " | positionAnchor: " + String(positionAnchor));

    // Extract the steps array from the JSON object
    JsonArray stepsArray = doc["steps"].as<JsonArray>();
    if (stepsArray.isNull()) {
        mqtt->print("Error: JSON does not contain a 'steps' array.");
        mqtt->reply_error(request, GrillConstants::ERROR_NO_STEPS);
        return;
    }

    // Reset value for steps count (in the loop we will set the good value)
    currentProgram.stepsCount = 0;
    
    for (JsonObject v : stepsArray) {
       
        if (currentProgram.stepsCount >= GrillConstants::MAX_PROGRAM_STEPS) break; // Array protection

        Step& s = currentProgram.steps[currentProgram.stepsCount];
        s.time = v[GrillConstants::JSON_TIME] | 0;
        s.temperature = v[GrillConstants::JSON_TEMPERATURE] | -1;
        // NO_TARGET (not -1) marks "not set", since relative mode allows negative position deltas.
        // (Copied to a local first: ArduinoJson's operator| binds its default by reference, which
        // would otherwise require an out-of-line definition for this header-only static constexpr.)
        const int noPositionTarget = GrillConstants::NO_TARGET;
        s.position = v[GrillConstants::JSON_POSITION] | noPositionTarget;
        s.rotation = v[GrillConstants::JSON_ROTATION] | -1;
        s.action = v[GrillConstants::JSON_ACTION] | "";
        
        currentProgram.stepsCount++;
    }

    // Init state variables
    programCurrentStep = 0;
    stepDurationStart = millis();
    
    // Init states machine
    programState = PROGRAM_RUNNING;
    stepState = STEP_STARTING;

    // Indicate the frontend that the program is being executed.
    publish_program_status();
}

void ProgramManager::finish_program(bool forcedCancelation = false) {

    // Depending if has been forced or not the color will deffer.
    if (forcedCancelation) {
        statusLed->pulse(3, CRGB::Orange1, 250, 250, LedState::OFF);
        mqtt->print("Program cancelled and system restarted."); 
    } else {
        statusLed->pulse(3, CRGB::Green, 200, 200, LedState::OFF);
        mqtt->print("Program completed successfully");
    }

    // Clear targets and stop actuator.
    movement->targetPosition = GrillConstants::NO_TARGET;
    movement->targetDegrees = GrillConstants::NO_TARGET;
    movement->targetTemperature = GrillConstants::NO_TARGET;
    movement->stop_lineal_actuator();
    
    // Clear the in-memory program
    programCurrentStep = 0;
    stepDurationStart = 0;
    
    // Reset state machine
    programState = PROGRAM_IDLE;
    stepState = STEP_COMPLETED;

    // Public final status
    publish_program_status();

}

void ProgramManager::update_program() {

    // Si no hay programa activo, salir
    if (programState != PROGRAM_RUNNING) {
        return;
    }

    // Blink the light to indicate a program is being executed.
    statusLed->setState(LedState::PROGRAM_RUNNING);
    
    // Verify if all the steps have been completed
    if (programCurrentStep >= currentProgram.stepsCount) {
        finish_program();
        return;
    }
    
    Step& currentStep = currentProgram.steps[programCurrentStep];
    
    // Máquina de estados para el paso actual
    switch (stepState) {
        case STEP_STARTING:
            start_current_step();
            break;
            
        case STEP_MOVING_TO_TARGET:
            check_target_reached();
            break;
            
        case STEP_WAITING_TIME:
            check_time_elapsed();
            break;
            
        case STEP_EXECUTING_ACTION:
            execute_current_action();
            break;
            
        case STEP_COMPLETED:
            advance_to_next_step();
            break;
    }
}

void ProgramManager::start_current_step() {
    
    Step& step = currentProgram.steps[programCurrentStep];
    
    stepStartUnix = Utils::get_current_unix_time();

    mqtt->print("Starting step " + String(programCurrentStep + 1) + "/" + String(currentProgram.stepsCount));
    
    // Verificar qué tipo de paso es
    if (step.action != "") {
        // Es una acción
        stepState = STEP_EXECUTING_ACTION;
    } else if (step.temperature != -1) {
        // Movimiento por temperatura
        movement->go_to_temp(step.temperature);
        stepState = STEP_MOVING_TO_TARGET;
    } else if (step.position != GrillConstants::NO_TARGET) {
        // Movimiento por posición (absoluta o relativa al punto de inicio del programa)
        int resolvedTarget = resolve_target_position(step.position);
        mqtt->print("Position step: raw=" + String(step.position) +
                    " mode=" + currentProgram.referenceType +
                    " anchor=" + String(positionAnchor) +
                    " -> resolvedTarget=" + String(resolvedTarget));
        movement->go_to(resolvedTarget);
        stepState = STEP_MOVING_TO_TARGET;
    } else if (step.rotation != -1) {
        // Movimiento por rotación
        movement->go_to_rotor(step.rotation);
        stepState = STEP_MOVING_TO_TARGET;
    } else {
        // Si es un paso de "Solo Tiempo" (sin movimientos ni acciones)
        // Pasamos directamente a esperar el tiempo.
        stepState = STEP_WAITING_TIME;
    }
}

void ProgramManager::check_target_reached() {
    // Verificar si MovementManager terminó el movimiento
    if (!movement->has_any_active_target()) {
        mqtt->print("Target reached for step " + String(programCurrentStep + 1));
        stepDurationStart = millis();
        stepState = STEP_WAITING_TIME;
    }
}

void ProgramManager::check_time_elapsed() {
    Step& step = currentProgram.steps[programCurrentStep];
    unsigned long stepElapsedTime = millis() - stepDurationStart;
    
    if (stepElapsedTime >= step.time * 1000) {
        mqtt->print("Time elapsed for step " + String(programCurrentStep + 1));
        stepState = STEP_COMPLETED;
    }
}

void ProgramManager::execute_current_action() {
    Step& step = currentProgram.steps[programCurrentStep];
    
    if (step.action == "flip") {
        mqtt->print("Executing flip action");
        movement->turn_around();
        stepState = STEP_MOVING_TO_TARGET;
    } else {
        mqtt->print("Unknown action: " + String(step.action));
        stepState = STEP_COMPLETED;
    }
}


void ProgramManager::advance_to_next_step() {
    mqtt->print("Step " + String(programCurrentStep + 1) + " completed");
    programCurrentStep++;
    stepState = STEP_STARTING;

    // We must notify that the step has changed.
    publish_program_status(); 
}

int ProgramManager::resolve_target_position(int stepPosition) {
    int target = stepPosition;

    if (currentProgram.referenceType == GrillConstants::PAYLOAD_REFERENCE_TYPE_RELATIVE) {
        target = positionAnchor + stepPosition;
    }

    // Clamp to the encoder's valid range regardless of mode.
    if (target < 0) target = 0;
    if (target > 100) target = 100;

    return target;
}

bool ProgramManager::is_program_running() {
    return programState == PROGRAM_RUNNING;
}

void ProgramManager::publish_program_status() {

    JsonDocument doc; 
    
    bool isRunning = is_program_running();
    doc["isRunning"] = isRunning;

    if (isRunning) {
        doc["name"] = currentProgram.name;
        doc["programId"] = currentProgram.id;
        doc["currentStepIndex"] = programCurrentStep;
        doc["referenceType"] = currentProgram.referenceType;

        // We manually build the steps array to inject runtime data
        JsonArray stepsArr = doc["steps"].to<JsonArray>();
        
        for (int i = 0; i < currentProgram.stepsCount; i++) {
            JsonObject sObj = stepsArr.add<JsonObject>();
            Step& s = currentProgram.steps[i];
            
            if (s.time != 0) sObj["time"] = s.time;
            if (s.temperature != -1) sObj["temperature"] = s.temperature;
            if (s.position != GrillConstants::NO_TARGET) sObj["position"] = s.position;
            if (s.rotation != -1) sObj["rotation"] = s.rotation;
            if (s.action != "") sObj["action"] = s.action;

            // If the step is the current one add step start timestamp
            if (i == programCurrentStep) {
                sObj["stepStartUnix"] = stepStartUnix;
            }
        }
    }

    String jsonOutput;
    serializeJson(doc, jsonOutput);
    
    String responseTopic = mqtt->parse_topic(GrillConstants::TOPIC_STATE_PROG_CURRENT);
    mqtt->publish_message(responseTopic, jsonOutput.c_str(), true);
}