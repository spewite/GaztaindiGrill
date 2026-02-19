
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

void GrillMQTT::print(String msg) {
    Serial.print("[");
    Serial.print(this->grillIndex);
    Serial.print("] ");
    Serial.println(msg);
    publish_message(parse_topic("log"), msg);
}

String GrillMQTT::parse_topic(String action) {
    return "grill/" + String(grillIndex) + "/" + action;
}

bool GrillMQTT::publish_message(const String& topic, const String& payload, bool retain) {
    if (!client.connected()) {
        extern void connect_to_mqtt();
        connect_to_mqtt();
    }
    return client.publish(topic.c_str(), payload.c_str(), retain);
}