#ifndef ETHERNET_NTP_H
#define ETHERNET_NTP_H

/*
-------------------------------------------------------------------------------------
EthernetNTP
  Non-blocking NTP client over the W5500's own UDP stack
  (arduino-libraries/Ethernet). configTime()/SNTP cannot be used because they run
  on the ESP32 lwIP stack, which has no route to the internet once WiFi is gone.

  The system clock is set in UTC (via settimeofday); local time is obtained
  through the TZ environment variable, so Utils::get_current_unix_time() and any
  localtime() calls keep working unchanged.

  update() must be called from loop(). It never blocks waiting for a reply: it
  sends a request and checks for the response on later calls. It performs an
  initial sync (retrying until it succeeds) and then re-syncs periodically.
-------------------------------------------------------------------------------------
*/

#include <Arduino.h>
#include <Ethernet.h>
#include <EthernetUdp.h>

class EthernetNTP {
public:
    // server: NTP hostname (resolved via DNS, then cached).
    // resyncIntervalMs: how often to re-sync after the first success.
    void begin(const char* server, unsigned long resyncIntervalMs = 3600000UL);

    // Call from loop(). Returns true if the clock was just updated.
    bool update();

    bool isSynced() const { return _synced; }

private:
    enum State { IDLE, WAITING };

    EthernetUDP   _udp;
    const char*   _server          = nullptr;
    IPAddress     _serverIP         = IPAddress(0, 0, 0, 0);
    unsigned long _resyncInterval   = 3600000UL;
    unsigned long _retryInterval    = 5000UL;   // between attempts until first sync
    unsigned long _lastAttempt      = 0;
    unsigned long _lastSync         = 0;
    unsigned long _sendTime         = 0;
    State         _state            = IDLE;
    bool          _synced           = false;
    bool          _begun            = false;

    void sendRequest();
};

#endif
