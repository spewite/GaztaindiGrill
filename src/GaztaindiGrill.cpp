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

unsigned long previousMillisTemp = 0; 
const long intervalTemp = 3000; // Temperaturan estadua aktualizatzeko pausa, MQTT ez kargatzeko.

// Funtziyuak lenuotik deklaratu, bestela erroria emateu.
void connectToWiFi();
void connectToMQTT();
void handleMQTTCallback(char* topic, byte* payload, unsigned int length);
bool publicarMQTT(const String& topic, const String& payload);

void setup() {
    Serial.begin(115200);
    SPI.begin();

    connectToWiFi();
    client.setCallback(handleMQTTCallback);
    connectToMQTT();

    for (int i = 0; i < NUM_GRILLS; ++i) {
        grills[i] = new Grill(i);
        if (grills[i]->setup_devices()) {
            Serial.println("Los dispositivos de la parrilla " + String(i) + " se han configurado correctamente");
            grills[i]->resetear_sistema();
            grills[i]->subscribe_topics();

        } else {
            Serial.println("Ha habido un error al configurar los dispositivos de la parrilla " + String(i));
        }
    }
}



void loop() {
    if (!client.connected()) {
        connectToMQTT();
    }
    client.loop(); 

    /// --------------------------------- ///
    ///       MANEJAR LAS PARADAS DE      /// 
    ///        LOS GO_TO / PROGRAMA       /// 
    /// --------------------------------- ///

    grills[0]->manejar_parada_rotor();
    grills[0]->manejar_parada_encoder(); 
    grills[0]->manejar_parada_temperatura(); 
    grills[0]->update_programa();

    /// -------------------------------- ///
    ///          HOME ASSISTANTEKO       /// 
    ///        ESTADUAK AKTUALIZATU      /// 
    /// -------------------------------- ///

    grills[0]->update_rotor_encoder();
    grills[0]->update_encoder();

    // Temperatura irakutzeko pausa, MQTT ez kargatzeko.
    unsigned long currentMillisTemp = millis();
    if (currentMillisTemp - previousMillisTemp >= intervalTemp) {
        // Guardar el tiempo actual
        previousMillisTemp = currentMillisTemp;

        grills[0]->update_temperature(); // Kontuan euki ezkerreko parrillak bakarrik eukikoula pt100
    }
}

/// -------------------------------- ///
///             MQTT & WIFI          /// 
/// -------------------------------- ///

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
        connectToMQTT();
    }
    return client.publish(topic.c_str(), payload.c_str());
}

void handleMQTTCallback(char* topic, byte* payload, unsigned int length) {
    char mensaje[length + 1];
    memcpy(mensaje, payload, length);
    mensaje[length] = '\0';

    // Akziyua, eta akziyua dagokion parrilla atea, formatua hau dala kontuan izanda: "grill/{id}/{accion}" 
    int id;
    char accion[120]; // MQTT topic batek ezin du array honen dimentsiyua baño handiyo izan.
    sscanf(topic, "grill/%d/%s", &id, accion);

    // Verificar que el id es válido antes de usarlo
    if (id >= 0 && id < NUM_GRILLS) {
        grills[id]->handleMQTTMessage(accion, mensaje);
    }
}
