#ifndef GRILL_MQTT_H
#define GRILL_MQTT_H

#include <ArduinoJson.h>
#include <PubSubClient.h>
#include <GrillConstants.h>

extern PubSubClient client;

// One incoming command, unwrapped from its envelope.
//
// Deliberately a per-request value passed by reference rather than state held in
// GrillMQTT: client.loop() can deliver several messages in a single iteration, so a
// "current request id" member would be overwritten by the next message and a deferred
// answer would reach the wrong client. Keeping the id and the replied flag in here
// makes that impossible.
struct GrillRequest {
    String id;              // requestId from the envelope, or PAYLOAD_REQUEST_ID_EVERYONE
    String command;         // action topic this request arrived on, echoed back in the answer
    String value;           // scalar payload ("up", "50"); empty when the payload is an object
    const char* raw;        // original payload, for handlers that parse the object themselves
    bool replied = false;   // set by reply_ok/reply_error/defer, checked by the dispatcher
};

class GrillMQTT {
public:
    GrillMQTT(int index);

    void subscribe_to_topics();
    void subscribe_to_system_topics();
    void print(String msg);
    String parse_topic(String action);
    bool publish_message(const String& topic, const String& payload, bool retain = false);

    // ------------- REQUEST / RESPONSE ------------- //

    // Unwraps the envelope once, in the dispatcher. A payload that is not JSON (mosquitto_pub,
    // an older client) still yields a usable request addressed to EVERYONE.
    static GrillRequest parse_request(const String& command, const char* payload);

    // Success is whatever did not reject, so handlers only spell out their failures and the
    // dispatcher calls reply_ok() for everything that came back without an answer.
    void reply_ok(GrillRequest& request);
    void reply_error(GrillRequest& request, const char* errorCode);

    // "The answer comes later." Silences the dispatcher's automatic reply_ok() so a deferred
    // outcome can answer with reply_to() instead, without two contradictory replies going out.
    void defer(GrillRequest& request);

    // Answers a request whose id was stored earlier, once its outcome is finally known.
    void reply_to(const String& requestId, const String& command, bool ok, const char* errorCode);

private:

    int grillIndex;

};

#endif
