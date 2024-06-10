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

class Grill {
public:
    Grill(int index);

    bool setup_devices();
    void resetear_sistema();

    int get_rotor_encoder_value();
    long get_encoder_real_value();
    long get_encoder_value();
    int  get_temperature();
    void update_rotor_encoder();
    void update_encoder();
    void update_temperature();

    void subir();
    void bajar();
    void parar();
    void voltear();   

    void rotar();
    void parar_rotor();

    void go_to(int posicion);
    void go_to_temp(int temperatura);
    void go_to_rotor(int grados);
    
    void handleMQTTMessage(const char* topic, const char* payload);
    void executeProgram(const char* program);
    void subscribe_topics();

    void manejar_parada_rotor();
    void manejar_parada_encoder();
    void manejar_parada_temperatura();
    void update_programa();

private:
    int index;
    int rotorVueltas;
    int lastRotorEncoderValue;
    int lastTemperatureValue;
    long lastEncoderValue;
    int gradosObjetivo;
    int temperaturaObjetivo;
    
    CytronMD* drive;
    DeviceEncoder* encoder;
    DeviceEncoder* rotorEncoder;
    DeviceRotorDrive* rotor;
    Adafruit_MAX31865 pt100;

    void resetear_encoder(DeviceEncoder* sel_encoder);
    bool limit_switch_pulsado();
    void imprimir(String msg);
    bool publicarMQTT(const String& topic, const String& payload);
    String parse_topic(String accion);

    int posicionObjetivo;  // Añadido para rastrear la posición objetivo

    // Variables para la ejecución del programa de pasos
    struct Paso {
        int tiempo;
        int temperatura;
        int posicion;
        const char* accion;
    };
    Paso pasos[10]; // Ajusta el tamaño según sea necesario
    int numPasos;
    int currentStep;
    bool stepInProgress;
    unsigned long tiempoInicioPaso;
    bool cancelarPrograma; // Bandera para cancelar el programa
};

#endif
