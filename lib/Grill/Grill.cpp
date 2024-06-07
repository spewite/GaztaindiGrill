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

Grill::Grill(int index) : index(index), encoder(nullptr), drive(nullptr), rotor(nullptr), pt100(PIN_SPI_CS_GRILL_PT[0]), lastEncoderValue(0), lastTemperatureValue(0) {}

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
    resetear_encoder();
    print_encoder();

    // RESETEAR ROTOR
    imprimir("Reseteando el  rotor");
    rotor->go(ROTOR_ON);
    while (!limit_switch_pulsado()) {}
    rotor->stop();
    imprimir("Dispositivos calibrados");  

    // // //////////// RESETEAR ACTUADOR LINEAL ////////////////
    subir();
    imprimir("Subiendo la parrilla");

    while (!esta_arriba()) {
      imprimir("no esta arriba");
    } 

    imprimir("esta arriba");
    parar();
}

long Grill::get_encoder_value() {
    long encoderValue = encoder->get_data();
    return encoderValue;
}

void Grill::print_encoder() {
    long encoderValue = get_encoder_value();
    if (encoderValue == 0 || encoderValue == lastEncoderValue) { return; }
    lastEncoderValue = encoderValue;

    String encoderValueStr = String(encoderValue);
    String stringTopicEncoder = parse_topic("posicion");
    Serial.println(encoderValue);
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
 

bool Grill::esta_arriba() {
  float old_pos;

  // do // Asegurarse que la lectura del encoder es correcto.
  // {
    old_pos = encoder->get_data();
    imprimir(String(old_pos));
  // } while (old_pos == 0);

  
  delay(1000);

  imprimir(String(old_pos));
  
  return (encoder->get_data() == old_pos);
}
  

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

void Grill::resetear_encoder() {
    encoder->reset_counter(0);
}

bool Grill::limit_switch_pulsado() {
    return digitalRead(PIN_CS_LIMIT_SWITCH[index]) == LOW;
}

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

// String Grill::decode_topic(String topic) {
//     int lastSlashIndex = topic.lastIndexOf('/');
//     if (lastSlashIndex == -1) {
//         return topic; // Devuelve una cadena vacía si no se encuentra '/'
//     }
//     return topic.substring(lastSlashIndex + 1);
// }

void Grill::handleMQTTMessage(const char* pAccion, const char* payload) {
    // Assuming the payload contains a float value
    String accion(pAccion);

    publicarMQTT(parse_topic("log"), "Ha llegado una accion a la parrila + " + String(index) + ": " + accion);

    if (accion == "subir") {
        subir();
        imprimir("SUBIENDOO");
    } else if (accion == "bajar") {
        bajar();
        imprimir("BAJAR");
    } else if (accion == "parar") {
        parar();
    } else if (accion == "reiniciar") {
        resetear_sistema();
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