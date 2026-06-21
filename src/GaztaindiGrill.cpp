#include <Arduino.h>  
#include <Wire.h>
#include <SPI.h>
#include <WiFi.h>
#include <PubSubClient.h>
#include <ArduinoOTA.h>
#include <time.h>

#include <GrillConstants.h>
#include <GRILL_config.h>
#include <GrillSystem.h>
#include <StatusLED.h>

// Network configuration (W5500 CS pin, static IP, MQTT credentials, OTA port)
// lives in GRILL_config.h / GRILL_config.cpp.

EthernetClient ethClient;
PubSubClient client(ethClient);
EthernetOTA ethOTA;
GrillSystem* grillSystem;
StatusLED statusLed;
unsigned long lastConnectedMillis = 0; // Track last time we were online

// Functions declared from the library, otherwise error.
void connect_to_wifi();
void connect_to_mqtt();
void setup_ota();
void handle_mqtt_callback(char* topic, byte* payload, unsigned int length);
void setup_time();

void setup() {

    Serial.begin(115200);
    SPI.begin();

    // Start status led
    statusLed.begin();

    // WIFI & MQTT connection
    connect_to_wifi();
    setup_ota();
    client.setCallback(handle_mqtt_callback);
    connect_to_mqtt();

    // Setup time
    setup_time();

    // Notify that the system is resetting
    client.publish(GrillConstants::TOPIC_RESET_STATUS, GrillConstants::PAYLOAD_RESETTING, true);
  
    // Initialize GrillSystem (blocking call)
    grillSystem = new GrillSystem();
    if (!grillSystem->initialize_system(&statusLed)) {
        Serial.println("Error initializing grill system");
        statusLed.setState(LedState::ERROR);
        return;
    }

    // Notify that all grills are reset and the system is ready
    client.publish(GrillConstants::TOPIC_RESET_STATUS, GrillConstants::PAYLOAD_RESET_READY, true);
  
}
bool ota_active = false;

void loop() {
    // Handle OTA updates
    ArduinoOTA.handle();

    if (ota_active) return; // Stop everything else if updating

    unsigned long currentMillis = millis();

    // Actualizamos el contador solo si TODO está conectado
    if (WiFi.status() == WL_CONNECTED && client.connected()) {
        lastConnectedMillis = currentMillis;
    } else {
        // Si llevamos demasiado tiempo desconectados (>30s), reset profundo del WiFi
        if (currentMillis - lastConnectedMillis > 30000) {
            Serial.println("Connectivity lost for >30s. Performing deep WiFi reset...");
            WiFi.disconnect(true, true);
            WiFi.mode(WIFI_OFF);
            delay(100);
            WiFi.mode(WIFI_STA);
            WiFi.config(local_IP, gateway, subnet, dns);
            WiFi.begin(ssid, password);
            lastConnectedMillis = currentMillis; // Reset timer to wait another 30s
        }
    }

    // Ensure the WiFi connection.
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi Disconnected. Reconnecting...");
        connect_to_wifi();
        return;
    }

    // Check the MQTT connection.
    if (!client.connected()) {
        Serial.println("MQTT Disconnected. Reconnecting...");
        connect_to_mqtt();
        return;
    }

    // Essential to maintain the MQTT connection and process messages.
    client.loop(); 
    
    // Here you can uncomment your additional logic.
    grillSystem->update();

    // Update the LED status to show that everything is OK.
    statusLed.update();

    // You can add a small delay to avoid saturating the loop.
    delay(10); 
}

/// -------------------------------- ///
///             MQTT & WIFI          /// 
/// -------------------------------- ///

void connect_to_wifi() {
    Serial.print("Connecting to Wifi...");
    statusLed.setState(LedState::CONNECTING_WIFI);

    WiFi.mode(WIFI_STA); // Station Mode
    if (!WiFi.config(local_IP, gateway, subnet, dns)) {
        Serial.println("STA Failed to configure");
    }

    WiFi.config(local_IP, gateway, subnet, dns);
    WiFi.begin(ssid, password);
    while (WiFi.status() != WL_CONNECTED) {
        delay(500);
        Serial.print(".");
        statusLed.update();
    } 
    Serial.println("WiFi connected: " + WiFi.localIP().toString());
}
void connect_to_mqtt() {
    
    statusLed.setState(LedState::CONNECTING_MQTT);
    client.setServer(mqttServer, mqttPort);
    client.setKeepAlive(8); // Time that will trigger LWT.

    unsigned long lastMqttAttempt = 0; // Variable to control the time of the last attempt

    while (!client.connected()) {
        statusLed.update(); 

        // Only attempt to connect every 5 seconds
        if (millis() - lastMqttAttempt > 5000) {
            lastMqttAttempt = millis(); // Updates the time of the last attempt
            
            Serial.print("Attempting MQTT connection...");
            
            // LWT configuration
            const char* willTopic     = GrillConstants::TOPIC_LWT;
            const char* onlineMessage = GrillConstants::PAYLOAD_LWT_ONLINE;
            const char* willMessage   = GrillConstants::PAYLOAD_LWT_OFFLINE;
            int willQoS               = 1;
            bool willRetain           = true;

            if (client.connect(
                "ESP32Client", 
                mqttUser, mqttPassword,
                willTopic, willQoS, willRetain, willMessage
            )) {
                
                Serial.println("connected to mqtt");
                client.publish(willTopic, onlineMessage, true);

                // Re-subscribe to all necessary topics
                if (grillSystem) {
                    grillSystem->resubscribe_all();
                }
                
            } else {
                Serial.print("failed, rc=");
                Serial.print(client.state());
                Serial.println(" try again in 5 seconds");
            }
        }
    }

    // Pulse 
    statusLed.pulse(3, CRGB::Green, 250, 250, LedState::OFF);

}

void handle_mqtt_callback(char* topic, byte* payload, unsigned int length) {
    char message[length + 1];
    memcpy(message, payload, length);
    message[length] = '\0';

    String topicStr(topic);

    // Remove the "grill/" prefix
    if (!topicStr.startsWith("grill/")) return;
    String remainder = topicStr.substring(6); // What remains after "grill/"

    // Is it for an individual grill? (The next character is a digit)
    if (remainder.length() > 0 && isDigit(remainder.charAt(0))) {
        int firstSlash = remainder.indexOf('/');
        if (firstSlash != -1) {
            // Extract ID
            int id = remainder.substring(0, firstSlash).toInt();
            // Extract the action (everything after the ID)
            String actionStr = remainder.substring(firstSlash + 1);

            if (id >= 0 && id < GrillConstants::NUM_GRILLS && grillSystem) {
                Grill* grill = grillSystem->get_grill(id);
                if (grill) {
                    grill->handle_mqtt_message(actionStr.c_str(), message);
                }
            }
        }
    }
    // If it's not a digit, it's a SYSTEM command (e.g., "current_mode" or "restart")
    else if (grillSystem) {
        grillSystem->handle_mqtt_message(topicStr.c_str(), message);
    }
}

void setup_ota() {
    // Hostname defaults to esp3232-[MAC]
    ArduinoOTA.setHostname("GaztaindiGrill-OTA");

    // No password by default
    // ArduinoOTA.setPassword("admin");

    ArduinoOTA.onStart([]() {
        ota_active = true;
        String type;
        if (ArduinoOTA.getCommand() == U_FLASH) {
            type = "sketch";
        } else { // U_SPIFFS
            type = "filesystem";
        }
        // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
        Serial.println("Start updating " + type);
    });

    ArduinoOTA.onEnd([]() {
        Serial.println("\nEnd");
    });

    ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
        Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
    });

    ArduinoOTA.onError([](ota_error_t error) {
        Serial.printf("Error[%u]: ", error);
        if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
        else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
        else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
        else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
        else if (error == OTA_END_ERROR) Serial.println("End Failed");
    });

    ArduinoOTA.begin();
    Serial.println("OTA ready.");
}

void setup_time() {
    // Configura los servidores NTP y la zona horaria.
    // "pool.ntp.org" es el servidor estándar. 
    // Los ceros finales son para el offset de hora de verano si no usas una cadena de zona horaria.
    configTime(0, 0, "pool.ntp.org", "time.nist.gov");
    
    // Configurar zona horaria de España (CET/CEST)
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1);
    tzset();
}