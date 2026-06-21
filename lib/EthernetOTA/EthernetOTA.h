#ifndef ETHERNET_OTA_H
#define ETHERNET_OTA_H

/*
-------------------------------------------------------------------------------------
EthernetOTA
  Minimal HTTP firmware-update endpoint that works over the W5500's own TCP/IP
  stack (arduino-libraries/Ethernet). Standard ArduinoOTA/espota cannot be used
  with the W5500 because it relies on the ESP32 lwIP stack, so this provides an
  equivalent "upload without USB" path.

  Protocol:
    POST /update            <- raw firmware binary in the request body
    (Content-Length must be set; curl --data-binary @firmware.bin does this)

  On success the device flashes the new image and reboots.
-------------------------------------------------------------------------------------
*/

#include <Arduino.h>
#include <Ethernet.h>

class EthernetOTA {
public:
    void begin(uint16_t port = 3232);

    // Call from loop(). Returns true while an upload is being processed
    // (so the caller can pause the rest of the loop). When an update finishes
    // successfully the device reboots from inside this call.
    bool handle();

    // Optional callbacks for status/LED feedback.
    void onStart(void (*cb)());
    void onProgress(void (*cb)(size_t done, size_t total));
    void onEnd(void (*cb)(bool success, const String& message));

    // Optional shared secret. If set, the request must include the header
    // "X-Auth: <token>" or it is rejected. Empty = no authentication.
    void setAuthToken(const String& token);

private:
    EthernetServer* _server = nullptr;
    String _authToken = "";

    void (*_onStart)() = nullptr;
    void (*_onProgress)(size_t, size_t) = nullptr;
    void (*_onEnd)(bool, const String&) = nullptr;

    bool handleClient(EthernetClient& client);
    void sendResponse(EthernetClient& client, int code, const String& body);
};

#endif
