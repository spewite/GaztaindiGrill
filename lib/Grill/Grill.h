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
// #include <Adafruit_MAX31865.h>  
#include <Adafruit_MAX31855.h>

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

enum DireccionDual {
    ARRIBA,
    QUIETO,
    ABAJO
};

class Grill {
public:
    Grill(int index);

    bool esta_arriba_dual;
    bool setup_devices();
    void resetear_sistema();
    bool esta_arriba();

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
    void rotar_horario();
    void rotar_antihorario();
    void parar_rotor();
    bool limit_switch_pulsado(const int cs_limit_switch);

    // ------------------- GO_TO ------------------ //
    void go_to(int posicion);
    void go_to_temp(int temperatura);
    void go_to_rotor(int grados);

    DireccionDual direccion_dual;
    Modo modo;

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
    Adafruit_MAX31855* thermocouple;

    int index;
    
    // ----------------- LAST VALUES --------------- //
    long lastEncoderValue;
    int lastRotorEncoderValue;
    int lastTemperatureValue;

    // ------------------- RESETS ------------------ //
    void resetear_rotor();
    void resetear_actuador_lineal();
    void resetear_encoder(DeviceEncoder* sel_encoder);

    // -------------------- MQTT ------------------- //
    void imprimir(String msg);
    String parse_topic(String accion);
    bool publicarMQTT(const String& topic, const String& payload);

    // -------------- GO_TO OBJETIBUAK ------------- //
    int temperaturaObjetivo;
    int gradosObjetivo;
    int posicionObjetivo;  

    // ----------------- PROGRAMAK ----------------- //
    void cancelar_programa();
    
    struct Paso {
        int tiempo;
        int temperatura;
        int posicion;
        int rotacion;
        const char* accion;
    };
    Paso pasos[25]; // Programa batek dazkan pausu maximuak
    int numPasosPrograma;
    int pasoActualPrograma;
    bool objetivoAlcanzado;
    bool cancelarPrograma; 
    unsigned long tiempoInicioPaso;
};

#endif
