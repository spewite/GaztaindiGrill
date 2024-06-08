#include <Arduino.h>
#include <GRILL_config.h>
#include <ArduinoJson.h>

#include <Grill.h>

// Wi-Fi credentials
extern const char* ssid;
extern const char* password;
extern const char* mqttServer;
extern const int mqttPort;
extern const char* mqttUser;
extern const char* mqttPassword;

extern PubSubClient client;

// Definir un tipo de función para la publicación MQTT
typedef bool (*MQTTPublishFunction)(const String&, const String&);


Grill::Grill(int index) 
    : index(index), encoder(nullptr), drive(nullptr), rotor(nullptr), pt100(PIN_SPI_CS_GRILL_PT[0]), lastEncoderValue(0), lastTemperatureValue(0), posicionObjetivo(-1) {}

bool Grill::setup_devices() {
    // ENCODER
    encoder = new DeviceEncoder(PIN_SPI_CS_GRILL_ENC[index]);
    if (!encoder->begin(PULSES_ENCODER_GRILL, DATA_INTERVAL_GRILL, false)) {
        imprimir("Error Begin Encoder " + String(index));
        return false;
    }
    
    // ACTUADOR LINEAL
    drive = new CytronMD(PWM_DIR, PIN_GRILL_PWM[index], PIN_GRILL_DIR[index]);

    // LIMIT SWITCH
    pinMode(PIN_CS_LIMIT_SWITCH[index], INPUT_PULLUP);

    // ROTOR
    rotor = new DeviceRotorDrive(PIN_CS_ROTOR[index]);

    // PT100
    pinMode(PIN_SPI_CS_GRILL_PT[0], OUTPUT);
    digitalWrite(PIN_SPI_CS_GRILL_PT[0], HIGH);
    if (!pt100.begin(MAX31865_4WIRE)) {
        imprimir("Error Begin Pt100");
        return false;
    }

    return true;
}

void Grill::resetear_sistema() {
    
    // //////////// RESETEAR ROTOR ////////////////

    // imprimir("Reseteando el rotor");
    // rotor->go(ROTOR_ON);

    // while (!limit_switch_pulsado()) {}

    // rotor->stop();

    // //////////// RESETEAR ACTUADOR LINEAL ////////////////
    subir(); 
    imprimir("Subiendo la parrilla");

    while (!limit_switch_pulsado()) { // Oain rotorran berdiña, beste switch bat jarri
        // imprimir("No esta arriba");
    }

    imprimir("esta arriba");
    parar();

    // //////////// RESETEAR ENCODER ////////////////
    resetear_encoder();
    print_encoder();

    imprimir("Dispositivos calibrados");  
}

/// -------------------------- ///
///          GET/PRINT         /// 
///         PERIFERICOS        /// 
/// -------------------------- ///

long Grill::get_encoder_value() {
    return 100 - encoder->get_data();
}

void Grill::print_encoder() {
    long encoderValue = get_encoder_value();
    if (encoderValue == 0 || encoderValue == lastEncoderValue) { return; }
    lastEncoderValue = encoderValue;

    String encoderValueStr = String(encoderValue);
    String stringTopicEncoder = parse_topic("posicion");
    Serial.println("Encoder " + String(index) + " = " + encoderValue);
    publicarMQTT(stringTopicEncoder, encoderValueStr);
}

int Grill::get_temperature() {
    float temperature;
    digitalWrite(PIN_SPI_CS_GRILL_PT[index], LOW);
    temperature = pt100.temperature(RNOMINAL, RREF);
    digitalWrite(PIN_SPI_CS_GRILL_PT[index], HIGH);
    return (int) temperature;
}

void Grill::print_temperature() {
    int temperature = get_temperature();
    if (temperature == lastTemperatureValue || temperature < 0) { return; }
    lastTemperatureValue = temperature;

    String temperatureStr = String(temperature);
    Serial.println("Temperature = " + temperatureStr);
    publicarMQTT(parse_topic("temperatura"), temperatureStr);
}
 
/// -------------------------///
///          ACTUADOR        /// 
///           LINEAL         /// 
/// -------------------------///

void Grill::subir() {
    drive->setSpeed(-255);
}

void Grill::bajar() {
    drive->setSpeed(255);
}

void Grill::parar() {
    drive->setSpeed(0);
}

void Grill::voltear() {
    imprimir("Dando la vuelta");
    delay(1000);
}

// bool Grill::esta_arriba() {
//     float old_pos;

//     old_pos = encoder->get_data();
//     delay(1000);

//     return (encoder->get_data() == old_pos);
// }
  

void Grill::go_to(int posicion) {

    // En la funcion manejarMovimiento(), que se llamada en loop, manejamos cuando tenemos que parar.
    if (posicion < 0 || posicion > 100) {
        imprimir("Posición fuera de rango");
        return;
    }
    posicionObjetivo = posicion;
    int currentPercentage = get_encoder_value();

    if (currentPercentage < posicion) {
        subir();
    } else if (currentPercentage > posicion) {
        bajar();
    }
}

void Grill::manejarMovimiento() {

    int currentPercentage = get_encoder_value();
    int margen = 2;

    // publicarMQTT("MANEJAR MOVIMIENTO", String(abs(posicionObjetivo)));

    if (abs(currentPercentage - posicionObjetivo) <= margen && posicionObjetivo>0) {
        parar();
        posicionObjetivo = -1;
    } 
}

/// -------------------------- ///
///         PERIFERICOS        /// 
/// -------------------------- ///

void Grill::resetear_encoder() {
    encoder->reset_counter(0);
}


bool Grill::limit_switch_pulsado() {
    return digitalRead(PIN_CS_LIMIT_SWITCH[index]) == LOW;
}


/// ----------------------///
///          MQTT         /// 
/// ----------------------///

void Grill::imprimir(String msg) {
    Serial.println(msg);
    publicarMQTT(parse_topic("log"), msg);
}

bool Grill::publicarMQTT(const String& topic, const String& payload) {
    if (!client.connected()) {
        extern void connectToMQTT();
        connectToMQTT();
    }
    return client.publish(topic.c_str(), payload.c_str());
}

String Grill::parse_topic(String accion) {
    return "grill/" + String(index) + "/" + accion;
}

void Grill::subscribe_topics() {
    if (!client.connected()) {
        Serial.println("Cliente MQTT no está conectado. No se pueden suscribir los tópicos.");
        return;
    }

    const char* topics[] = {"log", "subir", "bajar", "parar", "reiniciar", "dirigir", "establecer_posicion"};
    const int numTopics = sizeof(topics) / sizeof(topics[0]);

    for (int i = 0; i < numTopics; ++i) {
        String topic = parse_topic(topics[i]);
        if (client.subscribe(topic.c_str())) {
            Serial.println("Subscribed to: " + topic);
        } else {
            Serial.println("Failed to subscribe to: " + topic);
        }
    }
}

void Grill::handleMQTTMessage(const char* pAccion, const char* pPayload) {
    // Assuming the payload contains a float value
    String accion(pAccion);
    String payload(pPayload);

    publicarMQTT(parse_topic("mqtt_topic_listener"), "Ha llegado una accion a la parrila " + String(index) + ". " + accion) + ": " + payload + ")";

   if (accion == "dirigir") {
        if (payload == "subir") {
            subir();
            // go_to(100);  // Ejemplo: subir hasta el 100%
        } else if (payload == "bajar") {
            bajar();
            // go_to(0);  // Ejemplo: bajar hasta el 0%
        } else if (payload == "parar") {
            parar();
            // estadoMovimiento = MOVIMIENTO_IDLE;
            // imprimir("Movimiento detenido por comando MQTT");
        }
    } else if (accion == "establecer_posicion") {
        int posicion = payload.toInt();
        go_to(posicion);
    } else if (accion == "reiniciar") {
        imprimir("Reiniciando sistema");
    }
}

void Grill::executeProgram(const char* program) {
    StaticJsonDocument<2000> doc;
    DeserializationError error = deserializeJson(doc, program);

    if (error) {
        imprimir("Error al serializar JSON");
        return;
    }

    JsonArray pasos = doc["pasos"].as<JsonArray>();

    for (const auto& paso : pasos) {
        if (paso.containsKey("tiempo") && paso.containsKey("posicion")) {
            int tiempo = paso["tiempo"];
            float posicion = paso["posicion"];

            Serial.print("Executing step - Tiempo: ");
            Serial.print(tiempo);
            Serial.print(", Posicion: ");
            Serial.println(posicion);

            // go_to(posicion);
            delay(tiempo * 1000); // Convert seconds to milliseconds
        } else if (paso.containsKey("accion")) {
            const char* accion = paso["accion"];

            Serial.print("Executing step - Accion: ");
            Serial.println(accion);

            if (strcmp(accion, "voltear") == 0) {
                voltear();
            }
        }
    }
}


// String Grill::decode_topic(String topic) {
//     int lastSlashIndex = topic.lastIndexOf('/');
//     if (lastSlashIndex == -1) {
//         return topic; // Devuelve una cadena vacía si no se encuentra '/'
//     }
//     return topic.substring(lastSlashIndex + 1);
// }