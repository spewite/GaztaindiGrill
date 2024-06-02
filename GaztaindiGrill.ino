#include <SPI.h>
#include <GRILL_config.h>
#include <Adafruit_MAX31865.h>
#include <DeviceEncoder.h>
// #include <DeviceGrillDrive.h>
#include <CytronMotorDriver.h>
#include <DeviceRotorDrive.h>
#include <WiFi.h> 
#include <PubSubClient.h>
#include <ArduinoJson.h>

#define NUM_GRILLS 1
#define RREF 430.0
#define RNOMINAL 100.0

DeviceEncoder* encoder[NUM_GRILLS];
CytronMD* drive[NUM_GRILLS];
Adafruit_MAX31865 pt100(PIN_SPI_CS_GRILL_PT[0]);
DeviceRotorDrive* rotor;

// Wi-Fi credentials
const char* ssid = "EUSKALTEL2";
const char* password = "3313GAZTAINDI3313";

// MQTT broker details
// const char* mqttServer = "192.168.0.100"; 
// const int mqttPort = 1883;
// const char* mqttUser = "mqtt_user";
// const char* mqttPassword = "mqtt_user";
// const char* mqttTopic = "home/grill/position";

const char* mqttServer = "192.168.0.29"; 
const int mqttPort = 1883;
const char* mqttUser = "";
const char* mqttPassword = "";

const char* mqttTopic = "grill/position";
const char* comandosParrilla0 = "grill/comandosParrilla0";
const char* comandosParrilla1 = "grill/comandosParrilla1";

const char* programa = "{\"nombre\": \"Programa basico\",\"pasos\": [{\"tiempo\": 30,\"posicion\": 90},{\"tiempo\": 50,\"posicion\": 70},{\"accion\": \"voltear\"},{\"tiempo\": 30,\"posicion\": 100}]}";

WiFiClient wifiClient;
PubSubClient client(wifiClient);


//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
////////                                              ////////
////////                      HASI                    ////////
////////                                              ////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////


void setup() {
  Serial.begin(115200);
  SPI.begin();

  connectToWiFi();
  connectToMQTT();


  if (setupDevices()) {
    imprimir("Los dispositivos se han configurado correctamente");

    resetearSistema();

  } else {
    imprimir("Ha habido un error al configurar los dispositivos");
  }

}

void loop() {
  // Manejar la reconexión de MQTT y procesar cualquier mensaje que llegue.
  if (!client.connected()) {
    connectToMQTT();
  }
  client.loop();

  leer_reinicio();
  
  printEncoder(0);

  // static unsigned long lastEncoderReadTime = 0;
  // unsigned long currentMillis = millis();

  // if (currentMillis - lastEncoderReadTime >= 1000) {

  //   //// ENCDOER ////
  //   printEncoder(0);

  //   //// TEMPERATURA ////
  //   // printTemperature();

  //   lastEncoderReadTime = currentMillis;
  // }

}

bool setupDevices() {
  
  bool success;
  
  
  for (int i = 0; i < NUM_GRILLS; i++) {

    //////////// ENCODERS ////////////////

    encoder[i] = new DeviceEncoder(PIN_SPI_CS_GRILL_ENC[i]);

    if (!encoder[i]->begin(PULSES_ENCODER_GRILL, DATA_INTERVAL_GRILL, false)) {
        imprimir("Error Begin Encoder " + String(i));
        // success = false;  // No retornar inmediatamente para permitir cualquier otra operación de limpieza o registro necesario.
    }
    // encoder[i]->reset_counter(PULSES_ENCODER_GRILL);
    
    //////////// ACTUADORES LINEALES ////////////////

    drive[i] = new CytronMD(PWM_DIR, PIN_GRILL_PWM[i], PIN_GRILL_DIR[i]);


    //////////// LIMIT SWITCH ////////////////

    pinMode(PIN_CS_LIMIT_SWITCH[i], INPUT_PULLUP);

  }

  //////////// PT100 ////////////////
  
  // Inicialización del PT100
  pinMode(PIN_SPI_CS_GRILL_PT[0], OUTPUT);
  digitalWrite(PIN_SPI_CS_GRILL_PT[0], HIGH);
  if (!pt100.begin(MAX31865_4WIRE)) {
      imprimir("Error Begin Pt100");
      // success = false;
  }

  //////////// ROTOR ////////////////

  rotor = new DeviceRotorDrive(PIN_CS_ROTOR);

  return true;  
}

void resetearSistema()
{
  for (int i = 0; i < NUM_GRILLS; i++) 
  {
    // // //////////// RESETEAR ACTUADOR LINEAL ////////////////
    // subir(i);
    // imprimir("Subiendo la parrilla");

    // while (!estaArriba(i)) {
    //   imprimir("no esta arriba");
    // } 

    // imprimir("esta arriba");
    // parar(i);
    

    // //////////// RESETEAR ROTOR  ////////////////
    // imprimir("Reseteando el rotor");
    
    // rotor->go(ROTOR_ON);
    // while (!limitSwitchPulsado(i))  
    // {
    //   // No se esta pulsando el fin de carrera
    // }
    // rotor->stop();

    // imprimir("Dispositivos calibrados");
  }

  //////////// RESETEAR ENCODERS LINEAL ////////////////
  resetear_encoders();

}


//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////
////////                                              ////////
////////      LEER LOS DATOS DE LOS PERIFERICOS       ////////
////////        ENCODERS, PT100, MOTOR, ROTOR         ////////
////////                                              ////////
//////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////


//////////// ENCODERS ////////////////
long getEncoderValue(int i) {

  // digitalWrite(PIN_SPI_CS_GRILL_ENC[i], LOW); // Activar el chip select
  long  encoderValue = encoder[i]->get_counter(); // Leer el valor del encoder 
  // digitalWrite(PIN_SPI_CS_GRILL_ENC[i], HIGH); // Desactivar el chip select 
     
  return encoderValue;

  // for (int attempts = 0; attempts < 3; attempts++) { // Intenta hasta 3 veces
  //   digitalWrite(PIN_SPI_CS_GRILL_ENC[i], LOW); // Activar el chip select
  //   encoderValue = encoder[i]->get_counter(false); // Leer el valor del encoder
  //   digitalWrite(PIN_SPI_CS_GRILL_ENC[i], HIGH); // Desactivar el chip select

  //   // Verifica si el valor del encoder es dentro de un rango esperado
  //   // if (encoderValue >= 1 && encoderValue <= 101) {
  //   if (encoderValue != 0) {
  //     return encoderValue; // Retorna el valor si es válido
  //   }
  //   delay(10); // Pequeña pausa entre intentos
  // }
  
  // Serial.println("Fallo al leer valor del encoder válido tras varios intentos.");
  // return -20; // Devuelve un código de error o un valor específico si no se logra una lectura válida
}


void printEncoder(int i) {

  long encoderValue = getEncoderValue(i);

  if (encoderValue == 0) {return;}

  // Crear una cadena con el valor del encoder para publicar
  String encoderValueStr = String(encoderValue);

  // Publicar en MQTT y comprobar si fue exitoso
  String stringTopicEncoder = "grill/encoder" + String(i);
  
  Serial.println(encoderValue);
  publicarMQTT(stringTopicEncoder, encoderValueStr); 
}

//////////// PT100 ////////////////

float getTemperature() {
    float temperature;
    for (int attempts = 0; attempts < 3; attempts++) { // Intenta hasta 3 veces
        digitalWrite(PIN_SPI_CS_GRILL_PT[0], LOW);
        temperature = pt100.temperature(RNOMINAL, RREF);
        digitalWrite(PIN_SPI_CS_GRILL_PT[0], HIGH);

        if (temperature >= -100.0 && temperature <= 850.0) return temperature;
        delay(10); // Pequeña pausa entre intentos
    }
    Serial.println("Fallo al leer temperatura válida tras varios intentos.");
    return NAN; // Devuelve Not-A-Number si no se logra una lectura válida
}



void printTemperature() {
  float temperature = getTemperature();
  // Convertir la temperatura a una cadena para imprimir

  // Convertir la temperatura a String para publicar
  String temperatureStr = String(temperature);

  Serial.println("Temperature = " + temperatureStr);
  publicarMQTT("grill/temperature", temperatureStr);
}


///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
////////                                       ////////
////////              UTILIDADES               ////////
////////                                       ////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////


void leer_reinicio()
{
  // Leer el puerto serial solo si hay datos disponibles
  if (Serial.available() > 0) {
    String input = Serial.readStringUntil('\n'); // Leer la línea completa
    // Comprobar si el comando es "reiniciar"
    if (input.equalsIgnoreCase("reiniciar")) {
      Serial.println("Reiniciando ESP32...");
      ESP.restart(); // Reinicia el ESP32
    }
  }
}


void resetear_encoders() {
  for (int i = 0; i<NUM_GRILLS; i++)
  {
    encoder[i]->reset_counter(PULSES_ENCODER_GRILL);
    printEncoder(i);  
  }   
}

bool limitSwitchPulsado(int i)
{
  return digitalRead(PIN_CS_LIMIT_SWITCH[i]) == LOW;
}


void imprimir(String msg)
{
  Serial.println(msg);
  publicarMQTT("grill/log", msg);
} 


bool estaArriba(int i) {
  float old_pos;

  // do // Asegurarse que la lectura del encoder es correcto.
  // {
    old_pos = encoder[i]->get_data();
    imprimir(String(old_pos));
  // } while (old_pos == 0);

  
  delay(1000);

  imprimir(String(old_pos));
  
  return (encoder[i]->get_data() == old_pos);
}


///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
////////                                       ////////
////////          MOVIMIENTOS PARRILLA         ////////
////////                                       ////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

void voltear() 
{
  imprimir("Dando la vuelta"); 
  delay(1000);
}

void subir(int i)
{
  // Actuadore lineala buruz-bera dao.
  drive[i]->setSpeed(-255);
}

void bajar(int i)
{
  // Actuadore lineala buruz-bera dao.
  drive[i]->setSpeed(+255);
}

void parar(int i)
{
  drive[i]->setSpeed(0);
}

///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
////////                                       ////////
////////                  MQTT                 ////////
////////                                       ////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////

void connectToWiFi() {
  Serial.print("Conectándose a la WiFi con SSID: ");
  Serial.println(ssid);

  WiFi.begin(ssid, password);

  unsigned long startTime = millis();
  int maxTimeout = 30000;  // Tiempo máximo de espera en milisegundos
  unsigned long attemptInterval = 1000;  // Intervalo inicial de 1 segundo
  int attemptCount = 0;
  
  while (WiFi.status() != WL_CONNECTED) {
    attemptCount++;
    leer_reinicio();

    Serial.print("Intento de conexión número ");
    Serial.print(attemptCount);
    Serial.print(": esperando ");
    Serial.print(attemptInterval / 1000);
    Serial.println(" segundos...");

    delay(attemptInterval);

    // Incrementa el intervalo de intento de manera exponencial
    attemptInterval *= 2;  // Duplica el intervalo de tiempo para el próximo intento
    if (attemptInterval > 16000) {  // Limita el intervalo máximo a 16 segundos
      attemptInterval = 16000;
    }

    if (millis() - startTime > maxTimeout) {
      Serial.println("Tiempo de espera excedido, no se pudo conectar a la WiFi");
      break;  // Salir del bucle si se supera el tiempo máximo
    }
  }

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Conectado a la WiFi con éxito");
  } else {
    Serial.println("No se pudo conectar a la WiFi");
    ESP.restart();  // Reinicia el microcontrolador si aún no está conectado
  }
}

void connectToMQTT() {
  client.setServer(mqttServer, mqttPort);
  client.setCallback(handleMQTTMessage);

  unsigned long startTime = millis();  // Guardar el tiempo de inicio

  while (!client.connected()) {
    Serial.println("Connecting to MQTT...");

    if (client.connect("ESP32Client", mqttUser, mqttPassword)) {
      Serial.println("Connected to MQTT");

      // Suscribirse a varios tópicos
      client.subscribe(mqttTopic);
      client.subscribe(comandosParrilla0);
      client.subscribe(comandosParrilla1);
      
    } else {
      // Serial.println("Failed with state " + String(client.state()));
      // if (millis() - startTime > 5000) {  // Comprobar si han pasado más de 5000 ms (5 segundos)
      //   Serial.println("No se ha podido conectarse a MQTT en 5 segundos. Reiniciando...");
      //   ESP.restart();  // Reiniciar el ESP32
      // }
      delay(2000);
    }
  }
}



bool publicarMQTT(const String& topic, const String& payload) {
  if (!client.connected()) {
    // Intentar reconectar a MQTT si no está conectado
    connectToMQTT();
  }
  return client.publish(topic.c_str(), payload.c_str());
}

void handleMQTTMessage(char* topic, byte* payload, unsigned int length) {

  // Assuming the payload contains a float value
  char mensaje[length + 1];
  memcpy(mensaje, payload, length);
  mensaje[length] = '\0';

  ////////////////////////
  // PARRILLA IZQUIERDA //
  ////////////////////////

  if (strcmp(topic, comandosParrilla0) == 0) {
    
    if (strcmp(mensaje, "subir") == 0) {
        subir(0);
    } else if (strcmp(mensaje, "bajar") == 0) {
        bajar(0);
        imprimir("BAJAR");
        delay(1000);
    } else if (strcmp(mensaje, "parar") == 0) {
        parar(0);
    }

  } 

  ////////////////////////
  //  PARRILLA DERECHA  //
  ////////////////////////

  if (strcmp(topic, comandosParrilla1) == 1) {

    if (strcmp(mensaje, "arriba") == 0) {
        subir(1);
    } else if (strcmp(mensaje, "abajo") == 0) {
        bajar(1);
    } else if (strcmp(mensaje, "stop") == 0) {
        parar(1);
    }

  } 

}


void executeProgram(const char* program) {
  StaticJsonDocument<2000> doc;
  DeserializationError error = deserializeJson(doc, program);

  if (error) {
    Serial.print("deserializeJson() failed: ");
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

      // go_to(posicion);
      delay(tiempo * 1000); // Convert seconds to milliseconds
    } else if (paso.containsKey("accion")) {
      const char* accion = paso["accion"];

      Serial.print("Executing step - Accion: ");

      if (strcmp(accion, "voltear") == 0) {
        voltear();
      }
    }
  }
}


///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
////////                                       ////////
////////                BASURA                 ////////
////////                                       ////////
///////////////////////////////////////////////////////
///////////////////////////////////////////////////////
