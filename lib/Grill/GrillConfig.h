#ifndef GRILL_CONFIG_H
#define GRILL_CONFIG_H

// Board wiring and deployment config: which GPIO drives what, and where the
// device finds the network. Changes when the physical board or the deployment
// does; contrast with GrillConstants.h, which is protocol/behaviour tuning
// shared by the business logic regardless of board revision.

#include <Arduino.h>
#include <IPAddress.h>

//====================================================================================
//   ESP32 PINOUTS
//====================================================================================

// PINS 34, 35, 36, 39 INPUT ONLY!

const int PIN_STATUS_LED     = 32;

// SPI bus. Not passed explicitly anywhere: the board is wired to the ESP32's
// default VSPI pins, so the plain SPI.begin() call already uses these.
const int PIN_SPI_MISO       = 19;
const int PIN_SPI_MOSI       = 23;
const int PIN_SPI_SCK        = 18;

// SPI SELEC Pt100 MAX31865
const int PIN_SPI_CS_GRILL_PT    = 14;

// SPI SELEC Encoders
const int PIN_SPI_CS_GRILL_ENC[] = {17, 16}; // Ezkerra: CS2, Eskubi CS3
const int PIN_SPI_CS_ROTOR_ENC   = 5;     // CS: 1

// W5500 Ethernet chip-select
const int PIN_W5500_CS           = 2;


// Grill Linear Motor drive
const int PIN_GRILL_PWM[]        = {33, 26};
const int PIN_GRILL_DIR[]        = {25, 27};

// Rotor
const int PIN_EN3           = 12;
const int PIN_EN4           = 13;
const int PIN_ENB           = 21;

// Limit
const int PIN_CS_LIMIT_ROTOR      = 22;
const int PIN_CS_LIMIT_LINEAL []  = {4, 15};


//====================================================================================
//   NETWORK (Ethernet W5500 + MQTT + OTA)
//====================================================================================

// Locally-administered MAC for the W5500 (defined in GrillConfig.cpp).
extern byte ETH_MAC[];

// Static IP configuration (same address the device used over WiFi).
const IPAddress ETH_LOCAL_IP(192, 168, 1, 100);
const IPAddress ETH_GATEWAY (192, 168, 1, 1);
const IPAddress ETH_SUBNET  (255, 255, 255, 0);
const IPAddress ETH_DNS     (8, 8, 8, 8);

// MQTT broker
const char * const MQTT_SERVER   = "192.168.1.76";
const int          MQTT_PORT     = 1883;
const char * const MQTT_USER     = "gaztaindi";
const char * const MQTT_PASSWORD = "gaztaindi";

// HTTP OTA endpoint port (POST /update)
const int          OTA_PORT      = 3232;

//====================================================================================
//   ENCODERS
//====================================================================================
const long PULSES_ENCODER_GRILL  = 5344L; // Default: 2400L
extern float DATA_INTERVAL_GRILL[]; // Declare as extern (Assign value in GrillConfig.cpp)
const long PULSES_ENCODER_ROTOR  = 2400L;
extern float DATA_INTERVAL_ROTOR[]; // Declare as extern (Assign value in GrillConfig.cpp)

//====================================================================================
//   NTP
//====================================================================================
const char * const NTP_SERVER   = "1.es.pool.ntp.org";

#endif
