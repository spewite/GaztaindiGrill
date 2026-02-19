#ifndef GRILL_MQTT_H
#define GRILL_MQTT_H

#include <PubSubClient.h>
#include <GrillConstants.h>

extern PubSubClient client;

class GrillMQTT {
public:
    GrillMQTT(int index);

    void subscribe_to_topics();
    void print(String msg);
    String parse_topic(String action);
    bool publish_message(const String& topic, const String& payload, bool retain = false);

private:

    int grillIndex;

};

#endif
