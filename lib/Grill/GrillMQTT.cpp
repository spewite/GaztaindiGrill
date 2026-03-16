
#include <GrillMQTT.h>

extern PubSubClient client;

GrillMQTT::GrillMQTT(int index) : grillIndex(index) { }

void GrillMQTT::subscribe_to_topics() {

    if (!client.connected()) {
        Serial.println("MQTT client is not connected. Cannot subscribe to topics.");
        return;
    }

    // We subscribe to all action topics for this grill using a wildcard
    String action_topic = "grill/" + String(grillIndex) + "/action/#";

    if (client.subscribe(action_topic.c_str())) {
        Serial.println("Subscribed to: " + action_topic);
    } else {
        Serial.println("Failed to subscribe to: " + action_topic);
    }
}

void GrillMQTT::subscribe_to_system_topics() {
    
    if (!client.connected()) {
        Serial.println("MQTT client is not connected. Cannot subscribe to topics.");
        return;
    }

    // Subscribing to global system topics
    client.subscribe(GrillConstants::TOPIC_CMD_REQ_CURRENT_MODE);
    client.subscribe(GrillConstants::TOPIC_REQ_MODE_CHANGE);
    client.subscribe(GrillConstants::TOPIC_CURRENT_MODE);
    client.subscribe(GrillConstants::TOPIC_CMD_SYS_RESTART);
    client.subscribe(GrillConstants::TOPIC_RESET_STATUS);
    
    Serial.println("Subscribed to System MQTT Topics");
}

void GrillMQTT::print(String msg) {
    if (this->grillIndex == -1) {
        Serial.print("[SYSTEM] ");
    } else {
        Serial.print("[");
        Serial.print(this->grillIndex);
        Serial.print("] ");
    }
    Serial.println(msg);
    publish_message(parse_topic("log"), msg);
}

String GrillMQTT::parse_topic(String action) {
    if (this->grillIndex == -1) {
        return "grill/" + action;
    }
    return "grill/" + String(grillIndex) + "/" + action;
}

bool GrillMQTT::publish_message(const String& topic, const String& payload, bool retain) {
    if (!client.connected()) {
        extern void connect_to_mqtt();
        connect_to_mqtt();
    }
    return client.publish(topic.c_str(), payload.c_str(), retain);
}