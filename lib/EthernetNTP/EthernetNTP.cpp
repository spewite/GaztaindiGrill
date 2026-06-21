#include <EthernetNTP.h>
#include <Dns.h>
#include <sys/time.h>
#include <string.h>

namespace {
const uint16_t NTP_PORT          = 123;
const uint16_t NTP_LOCAL_PORT    = 8888;
const int      NTP_PACKET_SIZE   = 48;
const unsigned long SEVENTY_YEARS = 2208988800UL; // seconds between 1900 and 1970
const unsigned long REPLY_TIMEOUT = 1200UL;       // ms to wait for a reply
}

void EthernetNTP::begin(const char* server, unsigned long resyncIntervalMs) {
    _server         = server;
    _resyncInterval = resyncIntervalMs;
    _serverIP       = IPAddress(0, 0, 0, 0);
    _lastAttempt    = 0;
    _lastSync       = 0;
    _state          = IDLE;
    _synced         = false;
    _udp.begin(NTP_LOCAL_PORT);
    _begun = true;
    Serial.println("NTP: client ready (server " + String(server) + ")");
}

void EthernetNTP::sendRequest() {
    // Resolve and cache the server IP on first use.
    if (_serverIP == IPAddress(0, 0, 0, 0)) {
        DNSClient dns;
        dns.begin(Ethernet.dnsServerIP());
        IPAddress ip;
        if (dns.getHostByName(_server, ip, 3000) == 1) {
            _serverIP = ip;
        } else {
            Serial.println("NTP: DNS resolve failed for " + String(_server));
            return; // stay IDLE; retried later
        }
    }

    uint8_t pkt[NTP_PACKET_SIZE];
    memset(pkt, 0, sizeof(pkt));
    pkt[0] = 0b11100011; // LI = 3 (unsynced), VN = 4, Mode = 3 (client)
    pkt[1] = 0;          // Stratum
    pkt[2] = 6;          // Polling interval
    pkt[3] = 0xEC;       // Peer clock precision

    _udp.beginPacket(_serverIP, NTP_PORT);
    _udp.write(pkt, NTP_PACKET_SIZE);
    _udp.endPacket();

    _sendTime = millis();
    _state    = WAITING;
}

bool EthernetNTP::update() {
    if (!_begun) { return false; }

    unsigned long now = millis();

    if (_state == WAITING) {
        if (_udp.parsePacket() >= NTP_PACKET_SIZE) {
            uint8_t buf[NTP_PACKET_SIZE];
            _udp.read(buf, NTP_PACKET_SIZE);

            // Transmit timestamp (seconds since 1900) is at bytes 40..43.
            unsigned long secsSince1900 =
                  ((unsigned long)buf[40] << 24) | ((unsigned long)buf[41] << 16) |
                  ((unsigned long)buf[42] <<  8) |  (unsigned long)buf[43];

            _state = IDLE;

            if (secsSince1900 > SEVENTY_YEARS) {
                time_t unixTime = (time_t)(secsSince1900 - SEVENTY_YEARS);
                struct timeval tv;
                tv.tv_sec  = unixTime;
                tv.tv_usec = 0;
                settimeofday(&tv, nullptr);

                _synced   = true;
                _lastSync = now;
                Serial.println("NTP: synced (UTC unix " + String((uint32_t)unixTime) + ")");
                return true;
            }
            // Malformed/zero timestamp: fall through, will retry per backoff.
        } else if (now - _sendTime > REPLY_TIMEOUT) {
            _state       = IDLE; // no reply in time
            _lastAttempt = now;
        }
        return false;
    }

    // IDLE: decide whether a (re)sync is due.
    bool due = _synced
                 ? (now - _lastSync >= _resyncInterval)
                 : (_lastAttempt == 0 || now - _lastAttempt >= _retryInterval);
    if (!due) { return false; }

    _lastAttempt = now;
    sendRequest();
    return false;
}
