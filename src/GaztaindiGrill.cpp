#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Ethernet.h>
#include <PubSubClient.h>
#include <EthernetOTA.h>
#include <EthernetNTP.h>
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
EthernetNTP ntp;
GrillSystem* grillSystem;
StatusLED statusLed;

// Functions declared from the library, otherwise error.
void connect_to_ethernet();
void connect_to_mqtt();
void setup_ota();
void handle_mqtt_callback(char* topic, byte* payload, unsigned int length);
void setup_time();
void publish_time();

void setup() {

    Serial.begin(115200);
    SPI.begin();

    // Start status led
    statusLed.begin();

    // ETHERNET & MQTT connection
    connect_to_ethernet();
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
    // Handle OTA updates (HTTP over the W5500). Reboots internally on success.
    if (ethOTA.handle()) return;

    if (ota_active) return; // Stop everything else if updating

    // Renew DHCP lease if applicable (no-op with a static IP, but harmless).
    Ethernet.maintain();

    // Ensure the physical Ethernet link is up.
    if (Ethernet.linkStatus() == LinkOFF) {
        Serial.println("Ethernet cable disconnected. Waiting for link...");
        statusLed.setState(LedState::CONNECTING_INTERNET);
        statusLed.update();
        delay(200);
        return;
    }

    // Best-effort time sync over Ethernet (non-blocking). Runs even when the
    // MQTT broker is unreachable. Publish the new time if we are connected.
    if (ntp.update() && client.connected()) {
        publish_time();
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
///           MQTT & ETHERNET        ///
/// -------------------------------- ///

void connect_to_ethernet() {
    Serial.print("Starting Ethernet (W5500)...");
    statusLed.setState(LedState::CONNECTING_INTERNET);

    // Tell the Ethernet library which pin is the W5500 chip-select.
    Ethernet.init(PIN_W5500_CS);

    // Static configuration (same IP the device used over WiFi).
    Ethernet.begin(ETH_MAC, ETH_LOCAL_IP, ETH_DNS, ETH_GATEWAY, ETH_SUBNET);

    if (Ethernet.hardwareStatus() == EthernetNoHardware) {
        Serial.println("\nERROR: W5500 not detected! Check SPI wiring / CS (GPIO2) / power.");
        statusLed.setState(LedState::ERROR);
        return;
    }

    // Wait (with a timeout) for the physical link to come up.
    unsigned long start = millis();
    while (Ethernet.linkStatus() == LinkOFF && millis() - start < 15000) {
        delay(250);
        Serial.print(".");
        statusLed.update();
    }

    Serial.println("\nEthernet ready. IP: " + Ethernet.localIP().toString());
}
// Non-blocking single MQTT connection attempt (throttled to once every 5s).
// IMPORTANT: this must NOT block. setup()/loop() need to keep calling
// ethOTA.handle() so the device stays flashable even if the broker is down.
void connect_to_mqtt() {

    static unsigned long lastMqttAttempt = 0;

    if (client.connected()) { return; }

    // Throttle: at most one attempt every 5s (but allow the very first attempt).
    if (lastMqttAttempt != 0 && millis() - lastMqttAttempt < 5000) { return; }
    lastMqttAttempt = millis();

    statusLed.setState(LedState::CONNECTING_MQTT);
    statusLed.update();
    client.setServer(MQTT_SERVER, MQTT_PORT);
    client.setKeepAlive(8); // Time that will trigger LWT.

    Serial.print("Attempting MQTT connection...");

    // LWT configuration
    const char* willTopic     = GrillConstants::TOPIC_LWT;
    const char* onlineMessage = GrillConstants::PAYLOAD_LWT_ONLINE;
    const char* willMessage   = GrillConstants::PAYLOAD_LWT_OFFLINE;
    int willQoS               = 1;
    bool willRetain           = true;

    if (client.connect(
            "ESP32Client",
            MQTT_USER, MQTT_PASSWORD,
            willTopic, willQoS, willRetain, willMessage
        )) {

        Serial.println("connected to mqtt");
        client.publish(willTopic, onlineMessage, true);

        // Re-subscribe to all necessary topics
        if (grillSystem) {
            grillSystem->resubscribe_all();
        }

        // Make the (already synced) time visible to clients on reconnect.
        publish_time();

        statusLed.pulse(3, CRGB::Green, 250, 250, LedState::OFF);

    } else {
        Serial.print("failed, rc=");
        Serial.print(client.state());
        Serial.println(" try again in 5 seconds");
    }
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
    // HTTP OTA over the W5500 (POST /update on port 3232).
    // Optional: ethOTA.setAuthToken("changeme"); // require header "X-Auth: changeme"

    ethOTA.onStart([]() {
        ota_active = true;
        statusLed.setState(LedState::CONNECTING_INTERNET);
        Serial.println("OTA: start updating firmware");
    });

    ethOTA.onProgress([](size_t done, size_t total) {
        if (total > 0) {
            Serial.printf("OTA Progress: %u%%\r", (unsigned)(done * 100 / total));
        }
        statusLed.update();
    });

    ethOTA.onEnd([](bool success, const String& message) {
        if (success) {
            Serial.println("\nOTA: update complete");
        } else {
            ota_active = false;
            statusLed.setState(LedState::ERROR);
            Serial.println("\nOTA: failed - " + message);
        }
    });

    ethOTA.begin(OTA_PORT);
    Serial.println("OTA ready (HTTP POST /update on port " + String(OTA_PORT) + ").");
}

void setup_time() {
    // Time is synced over Ethernet via NTP (see EthernetNTP / ntp.update() in the
    // loop). The system clock is kept in UTC; the TZ env converts to local time
    // for any localtime() calls. The actual sync happens asynchronously once the
    // link is up, so this just configures the timezone and starts the client.
    setenv("TZ", "CET-1CEST,M3.5.0,M10.5.0/3", 1); // Spain (CET/CEST)
    tzset();

    ntp.begin(NTP_SERVER);
}

// Publishes the current UTC unix time (retained) so clients can read the
// synced clock. No-op until NTP has synced at least once.
void publish_time() {
    if (!ntp.isSynced() || !client.connected()) { return; }
    time_t nowUtc = time(nullptr);
    client.publish(GrillConstants::TOPIC_TIME, String((uint32_t)nowUtc).c_str(), true);
}