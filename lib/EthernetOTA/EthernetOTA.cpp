#include <EthernetOTA.h>
#include <Update.h>

namespace {

// arduino-libraries/Ethernet's EthernetServer does not implement the
// begin(uint16_t) method that the ESP32 core's abstract Server base declares as
// pure virtual, which makes EthernetServer non-instantiable on ESP32. This thin
// subclass implements it (delegating to the library's no-arg begin()).
class OtaEthernetServer : public EthernetServer {
public:
    explicit OtaEthernetServer(uint16_t port) : EthernetServer(port) {}
    void begin(uint16_t port = 0) override { (void)port; EthernetServer::begin(); }
};

// Reads one CRLF-terminated line from the client (without the trailing CRLF).
// Returns false on timeout/disconnect before any line was received.
bool readLine(EthernetClient& client, String& line, unsigned long timeoutMs) {
    line = "";
    unsigned long start = millis();
    while (millis() - start < timeoutMs) {
        if (client.available()) {
            char c = (char)client.read();
            if (c == '\n') { return true; }
            if (c != '\r') { line += c; }
            start = millis();
        } else if (!client.connected()) {
            return line.length() > 0;
        } else {
            delay(1);
        }
    }
    return false;
}

} // namespace

void EthernetOTA::begin(uint16_t port) {
    _server = new OtaEthernetServer(port);
    _server->begin();
    Serial.println("HTTP OTA endpoint ready on port " + String(port) + " (POST /update)");
}

void EthernetOTA::onStart(void (*cb)())                            { _onStart = cb; }
void EthernetOTA::onProgress(void (*cb)(size_t, size_t))           { _onProgress = cb; }
void EthernetOTA::onEnd(void (*cb)(bool, const String&))          { _onEnd = cb; }
void EthernetOTA::setAuthToken(const String& token)               { _authToken = token; }

bool EthernetOTA::handle() {
    if (!_server) { return false; }

    EthernetClient client = _server->available();
    if (!client) { return false; }

    return handleClient(client);
}

void EthernetOTA::sendResponse(EthernetClient& client, int code, const String& body) {
    String status = (code == 200) ? "200 OK"
                  : (code == 400) ? "400 Bad Request"
                  : (code == 401) ? "401 Unauthorized"
                  : (code == 404) ? "404 Not Found"
                                  : "500 Internal Server Error";
    client.print("HTTP/1.1 " + status + "\r\n");
    client.print("Content-Type: text/plain\r\n");
    client.print("Connection: close\r\n");
    client.print("Content-Length: " + String(body.length()) + "\r\n");
    client.print("\r\n");
    client.print(body);
    client.flush();
}

bool EthernetOTA::handleClient(EthernetClient& client) {
    // --- Request line: "POST /update HTTP/1.1" ---
    String requestLine;
    if (!readLine(client, requestLine, 5000)) {
        client.stop();
        return false;
    }

    int firstSpace  = requestLine.indexOf(' ');
    int secondSpace = requestLine.indexOf(' ', firstSpace + 1);
    String method = (firstSpace > 0) ? requestLine.substring(0, firstSpace) : "";
    String path   = (firstSpace > 0 && secondSpace > firstSpace)
                        ? requestLine.substring(firstSpace + 1, secondSpace)
                        : "";

    // --- Headers ---
    long contentLength = -1;
    String authHeader = "";
    String line;
    while (readLine(client, line, 5000)) {
        if (line.length() == 0) { break; } // end of headers
        String lower = line;
        lower.toLowerCase();
        if (lower.startsWith("content-length:")) {
            contentLength = line.substring(line.indexOf(':') + 1).toInt();
        } else if (lower.startsWith("x-auth:")) {
            authHeader = line.substring(line.indexOf(':') + 1);
            authHeader.trim();
        }
    }

    // --- Routing / validation ---
    if (method != "POST" || path != "/update") {
        sendResponse(client, 404, "Not found. Use: POST /update\n");
        client.stop();
        return false;
    }
    if (_authToken.length() > 0 && authHeader != _authToken) {
        sendResponse(client, 401, "Unauthorized\n");
        client.stop();
        return false;
    }
    if (contentLength <= 0) {
        sendResponse(client, 400, "Missing/invalid Content-Length\n");
        client.stop();
        return false;
    }

    // --- Begin flashing ---
    Serial.println("OTA: receiving " + String(contentLength) + " bytes...");
    if (_onStart) { _onStart(); }

    if (!Update.begin((size_t)contentLength)) {
        String err = "Update.begin failed: " + String(Update.errorString());
        Serial.println("OTA: " + err);
        sendResponse(client, 500, err + "\n");
        client.stop();
        if (_onEnd) { _onEnd(false, err); }
        return false;
    }

    size_t remaining = (size_t)contentLength;
    uint8_t buf[1024];
    unsigned long lastData = millis();

    while (remaining > 0) {
        int avail = client.available();
        if (avail > 0) {
            size_t toRead = avail < (int)sizeof(buf) ? (size_t)avail : sizeof(buf);
            if (toRead > remaining) { toRead = remaining; }
            int n = client.read(buf, toRead);
            if (n > 0) {
                if (Update.write(buf, n) != (size_t)n) {
                    String err = "Update.write failed: " + String(Update.errorString());
                    Serial.println("OTA: " + err);
                    Update.abort();
                    sendResponse(client, 500, err + "\n");
                    client.stop();
                    if (_onEnd) { _onEnd(false, err); }
                    return false;
                }
                remaining -= n;
                lastData = millis();
                if (_onProgress) { _onProgress((size_t)contentLength - remaining, (size_t)contentLength); }
            }
        } else {
            if (!client.connected()) { break; }
            if (millis() - lastData > 15000) { break; } // stalled transfer
            delay(1);
        }
    }

    if (remaining != 0) {
        String err = "Incomplete upload, missing " + String(remaining) + " bytes";
        Serial.println("OTA: " + err);
        Update.abort();
        sendResponse(client, 500, err + "\n");
        client.stop();
        if (_onEnd) { _onEnd(false, err); }
        return false;
    }

    if (!Update.end(true)) {
        String err = "Update.end failed: " + String(Update.errorString());
        Serial.println("OTA: " + err);
        sendResponse(client, 500, err + "\n");
        client.stop();
        if (_onEnd) { _onEnd(false, err); }
        return false;
    }

    Serial.println("OTA: success, rebooting...");
    sendResponse(client, 200, "OK, rebooting\n");
    client.stop();
    if (_onEnd) { _onEnd(true, "ok"); }

    delay(500);
    ESP.restart();
    return true; // not reached
}
