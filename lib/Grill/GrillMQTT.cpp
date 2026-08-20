
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
    client.subscribe(GrillConstants::TOPIC_CMD_SYS_EMERGENCY_STOP);
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


// ------------- REQUEST / RESPONSE ------------- //

GrillRequest GrillMQTT::parse_request(const String& command, const char* payload) {
    GrillRequest request;
    request.command = command;
    request.id = GrillConstants::PAYLOAD_REQUEST_ID_EVERYONE;
    request.raw = payload;

    JsonDocument doc;
    if (deserializeJson(doc, payload) != DeserializationError::Ok || !doc.is<JsonObject>()) {
        // Not an envelope at all: a bare payload from mosquitto_pub or an older client.
        // Keep it working, addressed to everyone since there is nobody to answer.
        request.value = payload;
        return request;
    }

    const char* id = doc[GrillConstants::JSON_REQUEST_ID];
    if (id) { request.id = id; }

    // Only scalar values are lifted out here. Object payloads (a whole program) are left for
    // the handler to parse from raw, so they are not serialised and re-parsed for nothing.
    JsonVariant value = doc[GrillConstants::JSON_VALUE];
    if (!value.isNull() && !value.is<JsonObject>() && !value.is<JsonArray>()) {
        request.value = value.as<String>();
    }

    return request;
}

void GrillMQTT::reply_ok(GrillRequest& request) {
    request.replied = true;
    reply_to(request.id, request.command, true, nullptr);
}

void GrillMQTT::reply_error(GrillRequest& request, const char* errorCode) {
    request.replied = true;
    reply_to(request.id, request.command, false, errorCode);
}

void GrillMQTT::defer(GrillRequest& request) {
    request.replied = true;
}

void GrillMQTT::reply_to(const String& requestId, const String& command, bool ok, const char* errorCode) {
    JsonDocument doc;
    doc[GrillConstants::JSON_REQUEST_ID] = requestId;
    doc[GrillConstants::JSON_COMMAND] = command;
    doc[GrillConstants::JSON_OK] = ok;
    if (errorCode) { doc[GrillConstants::JSON_ERROR] = errorCode; }

    String output;
    serializeJson(doc, output);

    // retain stays false: this answers one command, it is not grill state.
    publish_message(parse_topic(GrillConstants::TOPIC_STATE_RESULT), output);
}