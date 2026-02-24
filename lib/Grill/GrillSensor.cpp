
#include <GrillSensor.h>

GrillSensor::GrillSensor(int index, GrillMQTT* mqtt, HardwareManager* hardware, ModeManager* modeManager):
    grillIndex(index), mqtt(mqtt), hardware(hardware), modeManager(modeManager),
    lastEncoderValue(0), lastRotorEncoderValue(0), lastTemperatureValue(0) {}


// ------------- ENCODER ------------- //

long GrillSensor::get_encoder_value() {
    long encoderValue = hardware->encoder->get_data();

    if (encoderValue < 0) encoderValue = 1;
    if (encoderValue > 100) encoderValue = 100;
    if (encoderValue == 0) encoderValue = GrillConstants::ENCODER_ERROR;

    return encoderValue;
}

void GrillSensor::update_encoder() {
    long encoderValue = get_encoder_value();
    if (encoderValue == GrillConstants::ENCODER_ERROR || encoderValue == lastEncoderValue) { return; }
    lastEncoderValue = encoderValue;

    String encoderValueStr = String(encoderValue);
    String stringTopicEncoder = mqtt->parse_topic(GrillConstants::TOPIC_STATE_SENSOR_POSITION);
    Serial.println("Encoder " + String(grillIndex) + " = " + encoderValue);
    mqtt->publish_message(stringTopicEncoder, encoderValueStr, true);
}

bool GrillSensor::is_at_top()
{   
    return limit_switch_pressed(PIN_CS_LIMIT_LINEAL[grillIndex]);
}

bool GrillSensor::limit_switch_pressed(const int CS_LIMIT_SWITCH) {
    return digitalRead(CS_LIMIT_SWITCH) == LOW;
}


// ------------- ROTOR ENCODER ------------- //
 
int GrillSensor::get_rotor_encoder_value()
{  
    int rotorEncoderValue = hardware->rotorEncoder->get_data();
     
    if (rotorEncoderValue<0) { rotorEncoderValue+=360; }  
    if (rotorEncoderValue == 0) return lastRotorEncoderValue;
 
    return rotorEncoderValue;
}

void GrillSensor::update_rotor_encoder() { 

    int rotorEncoderValue = get_rotor_encoder_value();
    
    if (rotorEncoderValue == lastRotorEncoderValue) { return; }
    lastRotorEncoderValue = rotorEncoderValue;

    // Only prints every 5th value.
    if (rotorEncoderValue % 5 == 0)
    {
        Serial.println("Rotor Encoder = " + String(rotorEncoderValue));
        String topic = mqtt->parse_topic(GrillConstants::TOPIC_STATE_SENSOR_ROTATION);
        mqtt->publish_message(topic, String(rotorEncoderValue), true);
    }
}


// ------------- PT 100 ------------- //

int GrillSensor::get_temperature() {
    double temperature = hardware->thermocouple->readCelsius();
    if (isnan(temperature)) {
        mqtt->print("Error reading temperature!");
        return -1;
    }
    return (int) temperature;
}

void GrillSensor::update_temperature() {
    int temperature = get_temperature();
    if (temperature == lastTemperatureValue || temperature < 0) { return; }
    lastTemperatureValue = temperature;

    String temperatureStr = String(temperature);
    Serial.println("Temperature = " + temperatureStr);
    String topic = mqtt->parse_topic(GrillConstants::TOPIC_STATE_SENSOR_TEMP);
    mqtt->publish_message(topic, temperatureStr, true);
}

bool GrillSensor::is_valid_temperature(int temperature) 
{
    return (temperature != -1);
} 
