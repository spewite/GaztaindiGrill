#ifndef GRILL_H
#define GRILL_H

#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include <PubSubClient.h>

#include <GRILL_config.h>
#include <DeviceEncoder.h>
#include <CytronMotorDriver.h>
#include <Adafruit_MAX31865.h>
#include <DeviceRotorDrive.h>
 
// Definir RNOMINAL y RREF
#define RREF 430.0
#define RNOMINAL 100.0

extern PubSubClient client;

enum Modo {
    NORMAL,
    BURRUNTZI,
    DUAL
};

class Grill {
public:
    Grill(int index);

    bool setup_devices();
    void resetear_sistema();

    // ----------------- GETTERS ----------------- //
    int get_rotor_encoder_value();
    long get_encoder_real_value();
    long get_encoder_value();
    int  get_temperature();

    // ---------- HOME ASSISTANT UPDATE ---------- //
    void update_rotor_encoder();
    void update_encoder();
    void update_temperature();

    // ---------------- BASIKUAK ----------------- //
    void subir();
    void bajar();
    void parar();

    void voltear();   
    void rotar();
    void parar_rotor();

    // ------------------- GO_TO ------------------ //
    void go_to(int posicion);
    void go_to_temp(int temperatura);
    void go_to_rotor(int grados);

    // ---- MANEJAR PARADAS (GO_TO / PROGRAMA) ---- //
    void manejar_parada_rotor();
    void manejar_parada_encoder();
    void manejar_parada_temperatura();
    void update_programa();

    // ------------------- MQTT ------------------- //
    void handleMQTTMessage(const char* topic, const char* payload);
    void executeProgram(const char* program);
    void subscribe_topics();


private:
    
    CytronMD* drive;
    DeviceEncoder* encoder;
    DeviceEncoder* rotorEncoder;
    DeviceRotorDrive* rotor;
    Adafruit_MAX31865 pt100;

    int index;
    bool modo;
    int rotorVueltas;

    // ----------------- LAST VALUES --------------- //
    long lastEncoderValue;
    int lastRotorEncoderValue;
    int lastTemperatureValue;

    // ------------------- RESETS ------------------ //
    void resetear_rotor();
    void resetear_encoder(DeviceEncoder* sel_encoder);
    bool limit_switch_pulsado();

    // -------------------- MQTT ------------------- //
    String parse_topic(String accion);
    void imprimir(String msg);
    bool publicarMQTT(const String& topic, const String& payload);

    // -------------- GO_TO OBJETIBUAK ------------- //
    int temperaturaObjetivo;
    int gradosObjetivo;
    int posicionObjetivo;  

    // ----------------- PROGRAMAK ----------------- //
    struct Paso {
        int tiempo;
        int temperatura;
        int posicion;
        const char* accion;
    };
    Paso pasos[25]; // Programa batek dazkan pausu maximuak
    int numPasosPrograma;
    int pasoActualPrograma;
    bool pasoEnProgreso;
    bool cancelarPrograma; 
    unsigned long tiempoInicioPaso;
};

#endif
