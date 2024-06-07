#include <Arduino.h> 
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>

#include <GRILL_config.h>
#include <Grill.h>

#define NUM_GRILLS 1

const char* ssid = "EUSKALTEL2";
const char* password = "3313GAZTAINDI3313";
const char* mqttServer = "192.168.68.64"; 
const int mqttPort = 1883;
const char* mqttUser = "gaztaindi";
const char* mqttPassword = "gaztaindi";

WiFiClient wifiClient;
PubSubClient client(wifiClient);

Grill* grills[NUM_GRILLS];

// Declaración de funciones
void connectToWiFi();
void connectToMQTT();
void handleMQTTCallback(char* topic, byte* payload, unsigned int length);
bool publicarMQTT(const String& topic, const String& payload);

void setup() {
  Serial.begin(115200);
  SPI.begin();

  connectToWiFi();
  connectToMQTT();

  for (int i = 0; i < NUM_GRILLS; ++i) {
      grills[i] = new Grill(i);
      if (grills[i]->setup_devices()) {
          Serial.println("Los dispositivos de la parrilla " + String(i) + " se han configurado correctamente");
          grills[i]->resetear_sistema();
      } else {
          Serial.println("Ha habido un error al configurar los dispositivos de la parrilla " + String(i));
      }
  }
  client.setCallback(handleMQTTCallback);
}

void loop() {
  if (!client.connected()) {
      connectToMQTT();
  }
  client.loop(); 
    
  grills[0]->print_encoder();
  grills[0]->print_temperature();
}

void connectToWiFi() {
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
    }
    Serial.println("WiFi connected");
}

void connectToMQTT() {
    client.setServer(mqttServer, mqttPort);
    while (!client.connected()) {
        Serial.print("Attempting MQTT connection...");
        if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
            Serial.println("connected");
        } else {
            Serial.print("failed, rc=");
            Serial.print(client.state());
            Serial.println(" try again in 5 seconds");
            delay(5000);
        }
    }
}

bool publicarMQTT(const String& topic, const String& payload) {
    if (!client.connected()) {
        extern void connectToMQTT();
        connectToMQTT();
    }
    return client.publish(topic.c_str(), payload.c_str());
}

void handleMQTTCallback(char* topic, byte* payload, unsigned int length) {
    char mensaje[length + 1];
    memcpy(mensaje, payload, length);
    mensaje[length] = '\0';

    // Extraer id y accion del topic asumiendo el formato "grill/{id}/{accion}"
    int id;
    char accion[20]; // Asumiendo que el tamaño de 'accion' no superará los 20 caracteres
    sscanf(topic, "grill/%d/%s", &id, accion);

    publicarMQTT("grill/0", "Nuevo topic: " + String(topic));

    // Verificar que el id es válido antes de usarlo
    if (id >= 0 && id < NUM_GRILLS) {
        grills[id]->handleMQTTMessage(accion, mensaje);
    }
}
