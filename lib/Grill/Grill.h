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
    long get_encoder_value();
    int  get_temperature();
    void print_encoder();
    void print_temperature();
    bool esta_arriba();
    void subir();
    void bajar();
    void parar();
    void voltear();   
    void handleMQTTMessage(const char* topic, const char* payload);
    void executeProgram(const char* program);
    void subscribe_topics();

private:
    int index;
    long lastEncoderValue;
    int lastTemperatureValue;
    CytronMD* drive;
    DeviceEncoder* encoder;
    DeviceRotorDrive* rotor;
    Adafruit_MAX31865 pt100;

    void resetear_encoder();
    bool limit_switch_pulsado();
    void imprimir(String msg);
    bool publicarMQTT(const String& topic, const String& payload);
    String parse_topic(String accion);
    // String decode_topic(String accion);
};

#endif
