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

#define ENCODER_ERROR -9999.0 // Definir un valor especial que represente cuando el encoder lee mal un valor.
#define SIN_OBJETIVO -999 // Definir un valor cuando no se tiene ningun objetivo en go_to

// Definir un tipo de función para la publicación MQTT
typedef bool (*MQTTPublishFunction)(const String&, const String&);


Grill::Grill(int index) :
    index(index), 
    encoder(nullptr), rotorEncoder(nullptr), drive(nullptr), rotor(nullptr), pt100(PIN_SPI_CS_GRILL_PT), // PERIFERIKUAK
    lastEncoderValue(0), lastRotorEncoderValue(0), lastTemperatureValue(0), // LAST VALUES
    rotor_vueltas(0), direccion_dual(QUIETO), modo(NORMAL), esta_arriba_dual(false), // DATUAK
    posicionObjetivo(SIN_OBJETIVO), gradosObjetivo(SIN_OBJETIVO), temperaturaObjetivo(SIN_OBJETIVO), // GO_TO HELBURUAK
    numPasosPrograma(0), pasoActualPrograma(0), pasoEnProgreso(false), tiempoInicioPaso(0), cancelarPrograma(false) // PROGRAMAK
    {}

bool Grill::setup_devices() {

    // ENCODER
    encoder = new DeviceEncoder(PIN_SPI_CS_GRILL_ENC[index]);
    if (!encoder->begin(PULSES_ENCODER_GRILL, DATA_INTERVAL_GRILL, false)) {
        imprimir("Error Begin Encoder " + String(index));
        return false;
    }
    
    // ACTUADOR LINEAL
    drive = new CytronMD(PWM_DIR, PIN_GRILL_PWM[index], PIN_GRILL_DIR[index]);

    // ACTUADOR LINEAL LIMIT SWITCH (RESETEATZEKO)
    pinMode(PIN_CS_LIMIT_LINEAL[index], INPUT_PULLUP);

    // ROTOR ETA PT100 (EZKERREKUAK BAKARRIK DAZKAE)
    if (index == 0)
    {
        // Rotor
        rotor = new DeviceRotorDrive(PIN_CS_ROTOR);
        
        // Rotor Encoder
        rotorEncoder = new DeviceEncoder(PIN_SPI_CS_ROTOR_ENC);
        if (!rotorEncoder->begin(PULSES_ENCODER_ROTOR, DATA_INTERVAL_ROTOR, false)) {
            imprimir("Error Begin Rotor Encoder");
            return false;
        }

        // Pt100
        pinMode(PIN_SPI_CS_GRILL_PT, OUTPUT);
        digitalWrite(PIN_SPI_CS_GRILL_PT, HIGH);
        if (!pt100.begin(MAX31865_4WIRE)) {
            imprimir("Error Begin Pt100");
            return false;
        }
    }

    return true;
}

void Grill::resetear_sistema() {
    
    // ------------- RESETEAR ROTOR ------------- //
    if (index == 0)
    {
        resetear_rotor();
    }

    // ------------- RESETEAR ACTUADOR LINEAL ------------- //
    resetear_actuador_lineal();
    
    // ------------- RESETEAR ENCODER ------------- //
    resetear_encoder(encoder);
    update_encoder();

    imprimir("Dispositivos calibrados");  
}

/// -------------------------- ///
///          GET/PRINT         /// 
///         PERIFERICOS        /// 
/// -------------------------- ///

// ------------- ROTOR ENCODER ------------- //

int Grill::get_rotor_encoder_value()
{
    int rotorEncoderValue = abs(rotorEncoder->get_data());

    if (rotorEncoderValue == 0) return lastRotorEncoderValue;
    
    while (rotorEncoderValue>=360)
    {
        rotorEncoderValue = rotorEncoderValue % 360;
    }

    rotor_vueltas = abs(rotorEncoder->get_data())/360;

    return rotorEncoderValue;
}

void Grill::update_rotor_encoder() { 

    int rotorEncoderValue = get_rotor_encoder_value();

    if (rotorEncoderValue == lastRotorEncoderValue) { return; }
    lastRotorEncoderValue = rotorEncoderValue;

    // Solo va a imprimir en los multiplos de 5.
    if (rotorEncoderValue % 5 == 0)
    {
        Serial.println("Rotor Encoder = " + String(rotorEncoderValue));
        publicarMQTT(parse_topic("inclinacion"), String(rotorEncoderValue));
        publicarMQTT(parse_topic("vueltas"), String(rotor_vueltas));
    }
}

// ------------- ENCODER ------------- //

long Grill::get_encoder_real_value() {
    long encoderValue = encoder->get_data();

    if (encoderValue < 0) encoderValue = 1;
    if (encoderValue > 100) encoderValue = 100;
    if (encoderValue == 0) encoderValue = ENCODER_ERROR;

    return encoderValue;
}

long Grill::get_encoder_value() {
    return 100-get_encoder_real_value();
}

void Grill::update_encoder() {
    long encoderValue = get_encoder_real_value();
    if (encoderValue == ENCODER_ERROR || encoderValue == lastEncoderValue) { return; }
    lastEncoderValue = encoderValue;
    encoderValue = 100 - encoderValue;

    String encoderValueStr = String(encoderValue);
    String stringTopicEncoder = parse_topic("posicion");
    Serial.println("Encoder " + String(index) + " = " + encoderValue);
    publicarMQTT(stringTopicEncoder, encoderValueStr);
}

// ------------- PT 100 ------------- //

int Grill::get_temperature() {
    float temperature;
    digitalWrite(PIN_SPI_CS_GRILL_PT, LOW);
    temperature = pt100.temperature(RNOMINAL, RREF);
    digitalWrite(PIN_SPI_CS_GRILL_PT, HIGH);
    return (int) temperature;
}

void Grill::update_temperature() {
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
    if (modo == DUAL)
    {
        direccion_dual = ARRIBA;
    } else 
    {
        drive->setSpeed(-255);
    }
}

void Grill::bajar() {
     if (modo == DUAL)
    {
        direccion_dual = ABAJO;
    } else 
    {
        drive->setSpeed(255);
    }
}

void Grill::parar() {
     if (modo == DUAL)
    {
        direccion_dual = QUIETO;
    } else 
    {
        drive->setSpeed(0);
    }
}

void Grill::voltear() {
    imprimir("Dando la vuelta");
    int inclinacionActual = get_rotor_encoder_value();
    int inclinacionObjetivo = (inclinacionActual + 180) % 360;
    go_to_rotor(inclinacionObjetivo);
}

/// -------------------------///
///            ROTOR         /// 
/// -------------------------///

void Grill::rotar()
{
    rotor->go(ROTOR_ON);
}

void Grill::parar_rotor()
{
    rotor->stop();
}

/// -------------------------- ///
///            GO TOs          /// 
/// -------------------------- ///

// ------------- ROTOR ------------- //

void Grill::go_to_rotor(int grados) {

    if (grados < 0 || grados >= 360) {
        imprimir("Grados fuera de rango");
        return;
    }
    gradosObjetivo = grados;

    // En la funcion manejar_parada_rotor(), que se llamada en loop, manejamos cuando tenemos que parar.
    rotor->go(ROTOR_ON);
}

void Grill::manejar_parada_rotor() {
    
    int posicionActual = get_rotor_encoder_value();
    int margen = 0;

    if (posicionActual == gradosObjetivo && gradosObjetivo != SIN_OBJETIVO ) { 
        rotor->stop();
        gradosObjetivo = SIN_OBJETIVO;
    } 
}

// ------------- ACTUADOR LINEAL ------------- //

void Grill::go_to(int posicion) {

    if (posicion < 0) posicion = 0;
    if (posicion > 100) posicion = 99;
    
    posicionObjetivo = posicion;
    int currentPercentage = get_encoder_value();

    // En la funcion manejar_parada_encoder(), que se llamada en loop, manejamos cuando tenemos que parar.
    if (currentPercentage < posicion) {
        subir();
    } else if (currentPercentage > posicion) {
        bajar();
    }
}

void Grill::manejar_parada_encoder() {
    int currentPercentage = get_encoder_value();
    int margen = 2;

    if (abs(currentPercentage - posicionObjetivo) <= margen && posicionObjetivo != SIN_OBJETIVO ) {
        parar();
        posicionObjetivo = SIN_OBJETIVO;
        pasoEnProgreso = true; // Marca que el paso está en progreso después de alcanzar la posición
        tiempoInicioPaso = millis(); // Comienza a contar el tiempo ahora
    } 
}

// ------------- PT100 ------------- //

void Grill::go_to_temp(int temperatura) {

    temperaturaObjetivo = temperatura;
    int currentTemperature = get_temperature();

    // En la funcion manejar_parada_encoder(), que se llamada en loop, manejamos cuando tenemos que parar.
    if (currentTemperature < temperatura) {
        subir();
    } else if (currentTemperature > temperatura) {
        bajar();
    }
}

void Grill::manejar_parada_temperatura() {
    int currentTemperature = get_temperature();
    int margen = 2; // Ajusta el margen según sea necesario

    if (abs(currentTemperature - temperaturaObjetivo) <= margen && temperaturaObjetivo != SIN_OBJETIVO ) {
        parar();
        temperaturaObjetivo = SIN_OBJETIVO;
        pasoEnProgreso = true; // Marca que el paso está en progreso después de alcanzar la temperatura
        tiempoInicioPaso = millis(); // Comienza a contar el tiempo ahora
    } 
}

/// ------------------------------------  ///
///         PERIFERICOS EXTRA FUNC        /// 
/// ------------------------------------  ///

bool Grill::esta_arriba()
{
    return (modo == DUAL) ? esta_arriba_dual : limit_switch_pulsado(PIN_CS_LIMIT_LINEAL[index]);
}

void Grill::resetear_rotor()
{
    imprimir("Reseteando el rotor");
    rotor->go(ROTOR_ON);

    while (!limit_switch_pulsado(PIN_CS_LIMIT_ROTOR)) {}

    rotor->stop();
    resetear_encoder(rotorEncoder);
}

void Grill::resetear_actuador_lineal()
{
    subir(); 
    imprimir("Subiendo la parrilla");

    // Mensajia ez imprimitzen eoteko denboa guztiyan, pausa bat hilua blokeatu gabe (MQTT ENTZUN AHAL IZATEKO)
    unsigned long previousMessageMillis = 0;
    const long messageInterval = 1000; // 1 segunduko pausa

    while (!esta_arriba()) {

        unsigned long currentMillis = millis();
        if (currentMillis - previousMessageMillis >= messageInterval) {
            previousMessageMillis = currentMillis;
            imprimir("Reseteando actuador lineal...");
        }
        client.loop();
    }

    imprimir("esta arriba");
    parar();
}

void Grill::resetear_encoder(DeviceEncoder* selected_encoder) {
    // Parametrotik pasatzezaio zein encoder nahideun reseteatu, hola ezdet beste metodo bat inber RotorEncoder reseteatzeko.
    selected_encoder->reset_counter(0);
}

bool Grill::limit_switch_pulsado(const int CS_LIMIT_SWITCH) {
    return digitalRead(CS_LIMIT_SWITCH) == LOW;
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

    const char* topics[] = {"log", "reiniciar", "dirigir", "establecer_posicion", "ejecutar_programa", "cancelar_programa", "inclinar", "dirigir_rotor", "establecer_modo"};
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
    String accion(pAccion);
    String payload(pPayload);

    publicarMQTT(parse_topic("mqtt_topic_listener"), "Ha llegado una accion a la parrilla " + String(index) + ". " + accion + ": " + payload);

    if (accion == "dirigir") {
        if (payload == "subir") {
            subir();
        } else if (payload == "bajar") {
            bajar();
        } else if (payload == "parar") {
            parar();
        }
    }
    
    if (accion == "establecer_posicion") {
        int posicion = payload.toInt();
        go_to(posicion);
    }
    
    if (accion == "reiniciar") {
        imprimir("Reiniciando sistema");
    }
    
    if (accion == "ejecutar_programa") {
        executeProgram(pPayload);
    }
    
    if (accion == "cancelar_programa") {
        cancelarPrograma = true;
        imprimir("Programa cancelado");
    }
    
    if (accion == "inclinar")
    {
        int grados = payload.toInt();
        go_to_rotor(grados);
    }
    
    if (accion == "dirigir_rotor")
    {
        if (payload == "rotar") {
            rotar();
        }
        
        if (payload == "parar_rotor") {
            parar_rotor();
        } 
    }
    
    if (accion = "establecer_modo")
    {
        if (payload == "normal")
        {
            modo = NORMAL;
            parar();
            resetear_rotor();
        }
        
        if (payload == "burruntzi") 
        {
            modo = BURRUNTZI;
            parar();
            parar_rotor();
        }
        
        if (payload == "dual")
        {
            // Modo duala GaztaindiGrill.cpp (main)-etik kontrolatuko da.
            modo = DUAL;
        }
    }
}

void Grill::executeProgram(const char* program) {
    StaticJsonDocument<2000> doc;
    DeserializationError error = deserializeJson(doc, program);

    if (error) {
        imprimir("Error al serializar JSON");
        return;
    }

    // Imprimir el JSON recibido para depuración
    Serial.println("JSON recibido:");
    serializeJson(doc, Serial);
    Serial.println();

    JsonArray array = doc.as<JsonArray>();
    numPasosPrograma = array.size();
    
    for (int i = 0; i < numPasosPrograma; i++) {
        JsonObject paso = array[i];
        if (paso.containsKey("tiempo") && paso.containsKey("temperatura")) {
            pasos[i] = {paso["tiempo"], paso["temperatura"], -1, nullptr};
        } else if (paso.containsKey("tiempo") && paso.containsKey("posicion")) {
            pasos[i] = {paso["tiempo"], -1, paso["posicion"], nullptr};
        } else if (paso.containsKey("accion")) {
            pasos[i] = {-1, -1, -1, paso["accion"]};
        }
    }

    // Inicializar variables de estado
    pasoActualPrograma = 0;
    pasoEnProgreso = false;
    tiempoInicioPaso = millis();
    cancelarPrograma = false; // Reiniciar la bandera de cancelación al iniciar un nuevo programa
}



void Grill::update_programa() {
    if (cancelarPrograma) {
        parar();
        return; // Salir si el programa ha sido cancelado
    }

    if (pasoActualPrograma >= numPasosPrograma) {
        return; // No hay más pasos que ejecutar
    }

    Paso& paso = pasos[pasoActualPrograma];

    if (!pasoEnProgreso) {
        if (paso.accion) {
            Serial.print("Executing step - Accion: ");
            Serial.println(paso.accion);

            if (strcmp(paso.accion, "voltear") == 0) {
                voltear();
            }

            pasoActualPrograma++;
        } else {
            if (paso.temperatura != -1) {
                go_to_temp(paso.temperatura);
                return; // Salir de la función para esperar a que se alcance la temperatura
            } else if (paso.posicion != -1) {
                go_to(paso.posicion);
                return; // Salir de la función para esperar a que se alcance la posición
            }
        }
    } else {
        unsigned long tiempoTranscurridoPaso = millis() - tiempoInicioPaso;
        int tiempo = paso.tiempo;

        if (tiempoTranscurridoPaso >= tiempo * 1000) {
            pasoEnProgreso = false;
            pasoActualPrograma++;
        }
    }
}
