
#include <HardwareManager.h>

// rotor, rotorEncoder and thermocouple are only allocated for grill 0 in setup_devices().
// They must start as nullptr so the guards elsewhere can tell "absent" from garbage.
HardwareManager::HardwareManager(int index, GrillMQTT* mqtt):
    mqtt(mqtt), drive(nullptr), encoder(nullptr), rotorEncoder(nullptr),
    rotor(nullptr), thermocouple(nullptr), grillIndex(index) {}

bool HardwareManager::setup_devices() {
   
    mqtt->print("Configuring devices for grill " + String(grillIndex));

    // ENCODER
    encoder = new DeviceEncoder(PIN_SPI_CS_GRILL_ENC[grillIndex]);
    if (!encoder->begin(PULSES_ENCODER_GRILL, DATA_INTERVAL_GRILL, false)) {
        mqtt->print("Error Begin Encoder " + String(grillIndex));
        return false;
    }
    
    // LINEAL ACTUATOR
    drive = new CytronMD(PWM_DIR, PIN_GRILL_PWM[grillIndex], PIN_GRILL_DIR[grillIndex]);

    // LINEAL ACTUATOR LIMIT SWITCH (RESET)
    pinMode(PIN_CS_LIMIT_LINEAL[grillIndex], INPUT_PULLUP);
    
    // ROTOR LIMIT SWITCH (RESET)
    pinMode(PIN_CS_LIMIT_ROTOR, INPUT_PULLUP);
    
    // ROTOR AND PT100 (LEFT SIDE ONLY)
    if (grillIndex == 0)
    {
        // Rotor
        rotor = new DeviceRotorDrive(PIN_EN3, PIN_EN4, PIN_ENB);
        
        // Rotor Encoder
        rotorEncoder = new DeviceEncoder(PIN_SPI_CS_ROTOR_ENC);
        if (!rotorEncoder->begin(PULSES_ENCODER_ROTOR, DATA_INTERVAL_ROTOR, true)) {
            mqtt->print("Error Begin Rotor Encoder");
            return false;
        }

        // Thermocouple
        // pinMode(PIN_SPI_CS_GRILL_PT, OUTPUT);
        // digitalWrite(PIN_SPI_CS_GRILL_PT, HIGH);
        // thermocouple = new Adafruit_MAX31855(PIN_SPI_SCK, PIN_SPI_CS_GRILL_PT, PIN_SPI_MISO);
        // SPI.beginTransaction((SPISettings(1000000, MSBFIRST, SPI_MODE0))); // Use SPI settings
        // if (!thermocouple->begin()) {
        //     print("Error Begin Thermocouple");
        //     return false;
        // }
    }

    return true;
}

void HardwareManager::reset_encoder(DeviceEncoder* selected_encoder) {
    selected_encoder->reset_counter(PULSES_ENCODER_GRILL);
}

void HardwareManager::reset_rotor_encoder() {
    rotorEncoder->reset_counter(0);
}
