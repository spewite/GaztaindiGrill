#include <ProgramManager.h>

ProgramManager::ProgramManager(int index, GrillMQTT* mqtt, MovementManager* movement, StatusLED* statusLed) : 
    grillIndex(index),
    mqtt(mqtt),
    movement(movement),
    statusLed(statusLed),
    programStepsCount(0),
    programCurrentStep(0),
    stepDurationStart(0),
    programId(-1),
    description(""),
    programName(""),
    creatorName(""),
    usageCount(-1)
     {}

    
void ProgramManager::execute_program(const char* program) { 
     
    // Cancelar cualquier programa anterior.
    finish_program(true);

    // Usamos JsonDocument para que v7 gestione la memoria
    DeserializationError error = deserializeJson(currentProgramJson, program);

    if (error) {
        mqtt->print("Error deserializing the program JSON.");
        return;
    }
    
    // Extract the program Id. We will use -1 if doesnt exist.
    programId = currentProgramJson["programId"] | -1;
    mqtt->print("Executing program with ID: " + String(programId));

    // Extract the steps array from the JSON object
    JsonArray stepsArray = currentProgramJson["steps"].as<JsonArray>();
    if (stepsArray.isNull()) {
        mqtt->print("Error: JSON does not contain a 'steps' array.");
        return;
    }
    
    programStepsCount = stepsArray.size(); 
    
    for (int i = 0; i < programStepsCount; i++) {
        JsonObject step = stepsArray[i];
        
        Step newStep = {
            .time = step[GrillConstants::JSON_TIME] | 0,
            .temperature = step[GrillConstants::JSON_TEMPERATURE] | -1,
            .position = step[GrillConstants::JSON_POSITION] | -1,
            .rotation = step[GrillConstants::JSON_ROTATION] | -1,
            .action = step[GrillConstants::JSON_ACTION] | nullptr
        };
        
        steps[i] = newStep;
    }

    // Set the metadata of the program
    description = currentProgramJson["description"] | "";
    programName = currentProgramJson["name"] | "";
    creatorName = currentProgramJson["creatorName"] | "";
    usageCount  = currentProgramJson["usageCount"] | -1;

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
    currentProgramJson.clear();
    programId = -1;
    programStepsCount = 0;
    programCurrentStep = 0;
    stepDurationStart = 0;
    
    // Reset state machine
    programState = PROGRAM_IDLE;
    stepState = STEP_COMPLETED;

    // Clear the in-memory program
    currentProgramJson.clear();

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
    if (programCurrentStep >= programStepsCount) {
        finish_program();
        return;
    }
    
    Step& currentStep = steps[programCurrentStep];
    
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
    
    Step& step = steps[programCurrentStep];
    
    mqtt->print("Starting step " + String(programCurrentStep + 1) + "/" + String(programStepsCount));
    
    // Verificar qué tipo de paso es
    if (step.action != nullptr) {
        // Es una acción
        stepState = STEP_EXECUTING_ACTION;
    } else if (step.temperature != -1) {
        // Movimiento por temperatura
        movement->go_to_temp(step.temperature);
        stepState = STEP_MOVING_TO_TARGET;
    } else if (step.position != -1) {
        // Movimiento por posición
        movement->go_to(step.position);
        stepState = STEP_MOVING_TO_TARGET;
    } else if (step.rotation != -1) {
        // Movimiento por rotación
        movement->go_to_rotor(step.rotation);
        stepState = STEP_MOVING_TO_TARGET;
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
    Step& step = steps[programCurrentStep];
    unsigned long stepElapsedTime = millis() - stepDurationStart;
    
    if (stepElapsedTime >= step.time * 1000) {
        mqtt->print("Time elapsed for step " + String(programCurrentStep + 1));
        stepState = STEP_COMPLETED;
    }
}

void ProgramManager::execute_current_action() {
    Step& step = steps[programCurrentStep];
    
    if (strcmp(step.action, "flip") == 0) {
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


void ProgramManager::publish_program_status() {
    
    JsonDocument doc; 
    
    bool isActive = (programState == PROGRAM_RUNNING);
    doc["isRunning"] = isActive;

    if (isActive) {
        doc["name"] = programName;
        doc["programId"] = programId;
        doc["currentStepIndex"] = programCurrentStep;
        doc["description"] = description;
        doc["creatorName"] = creatorName;
        doc["usageCount"] = usageCount;

        // Add the elapsed time of the current step, if applicable
        if (stepState == STEP_WAITING_TIME) {
            unsigned long elapsedTimeMs = millis() - stepDurationStart;
            doc["elapsedTime"] = elapsedTimeMs / 1000;
        } else {
            doc["elapsedTime"] = 0;
        }

        // Add the program
        doc["steps"] = currentProgramJson["steps"];
    }

    String jsonOutput;
    serializeJson(doc, jsonOutput);

    String responseTopic = mqtt->parse_topic(GrillConstants::TOPIC_STATE_PROG_CURRENT);
    mqtt->publish_message(responseTopic, jsonOutput.c_str(), true);

    mqtt->print("Published program status.");
}
