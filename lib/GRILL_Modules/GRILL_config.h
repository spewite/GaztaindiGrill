/*
-------------------------------------------------------------------------------------
GRILL_Config.h
  This Header define all Application's defaults 
  configurattion parameters.
-------------------------------------------------------------------------------------
*/

#ifndef DL_Configh
#define DL_Configh

#include <Arduino.h> 

//====================================================================================
//   ESP32 PINOUTS
//====================================================================================
// PINS SD CARD (DON'T CHANGE: FIRMWARE CODEC)
const int PIN_SPI_MISO       = 19;
const int PIN_SPI_MOSI       = 23;
const int PIN_SPI_SCK        = 18;

// SPI SELEC Pt100 MAX31865
const int PIN_SPI_CS_GRILL_PT[]  = {13, 22};
// SPI SELEC Encoders
const int PIN_SPI_CS_GRILL_ENC[] = {2, 4, 16}; // Default: 2, 12, 13
const int PIN_SPI_CS_ROTOR_ENC   = 15;  
// Grill Linear Motor drive
const int PIN_GRILL_PWM[]        = {33, 25}; //  {25, 32}
const int PIN_GRILL_DIR[]        = {26, 32}; //  {26, 33}
 
const int PIN_CS_ROTOR[]         = {14, 12};
const int PIN_CS_LIMIT_SWITCH[]  = {27, 0};
// const int PIN_CUALQUIERCOSA      = 00;

//====================================================================================
//   ENCODERS
//====================================================================================
const long PULSES_ENCODER_GRILL  = 5640L; // Default: 2400L // Neria: 5640L
extern float DATA_INTERVAL_GRILL[]; // Declarar como extern (Se le da valor en el Grill_config.cpp)
const long PULSES_ENCODER_ROTOR  = 2400L;
extern float DATA_INTERVAL_ROTOR[]; // Declarar como extern (Se le da valor en el Grill_config.cpp)
 
const uint8_t PIN_OLED_I2C_SDA  = 4;
const uint8_t PIN_OLED_I2C_SLC  = 15;
const uint8_t PIN_OLED_RESET    = 16;
// PIN BUTTOM
const uint8_t PIN_BUTTOM        = 2;
// PIN I2C BUS
const uint8_t I2C_BUS_0         = 0;
const uint8_t PIN_I2C0_SDA      = 33;
const uint8_t PIN_I2C0_SLC      = 32;
// PIN SERIAL
const uint8_t PIN_SERIAL_RX     = 26;
const uint8_t PIN_SERIAL_TX     = 27;

//====================================================================================
//   APPLICATION DEFAULT PARAMETERS
//====================================================================================
// NTP
const char * const NTP_SERVER   = "1.es.pool.ntp.org";
const int NTP_LOCAL_GMT         = 1;
const int NTP_SYNC_TRIES        = 5;
// CHANELS
const long BLUETOOH_BAUDS       = 9600L;
const long SERIAL_BAUDS         = 9600L;
const int TCP_PORT              = 5011;
const uint32_t SERIAL_CONF      = SERIAL_8N1;
// TIME CTES
const long TIME_WATHDOG         = (20 * 1000L);
const long TIME_DELAY_SECURE    = 5000L;
const long TIME_LOOP            = 500L;
const long TIME_REFRESH_DISPLAY	= 1000L;
const long TIME_CLEAR_DISPLAY  	= (TIME_REFRESH_DISPLAY * 60);
const long TIME_READ_MIN_RTC    = 500L;
const long TIME_READ_MIN_SENSOR = 1000L;
const long TIME_LOG_SENSOR      = 10000L; 
const long TIME_SYNC_LOG        = (2 * 60 * 1000L);
const long TIME_MIN_BUTTOM_ON   = 100L;
const long TIME_WAIT_WIFI       = 30000L;
const long TIME_WAIT_CHOICE     = 5000L;
// Configuration sd files
const char * const DIR_CONFIG 	= "/CONFIG";
const char * const DIR_DATA     = "/DATA";
const char * const DIR_LOG		= "/LOGS";
const char * const SUF_CONFIG   = "CF";
const char * const SUF_DATA  	= "CSV";
const char * const SUF_LOG  	= "LOG";
const int MAX_CONF_SD						= 10;
// Formats
const char * const FORMAT_DATE       = "%d/%02d/%02d";
const char * const FORMAT_TIME       = "%2d:%02d:%02d";
const char * const FORMAT_TEMP_OK    = "%5.1f";
const char * const FORMAT_PRES_OK    = "%7.1f";
const char * const FORMAT_TEMP_ERROR = "---.-";
const char * const FORMAT_PRES_ERROR = "-----.-";
const char CSV_SEP                   = ';';
// SENSOR
const int MAX_TYPE_SENSOR         = 3;
const int SENSOR_TYPE_BMP388      = 1;
const int SENSOR_TYPE_LPS25HB     = 2;
const int SENSOR_TYPE_MS5611      = 3;
const int SENSOR_TYPE_DPS310      = 4;
const int SENSOR_TYPE_HDC1080     = 5;
const int SENSOR_TYPE_TMP117      = 6;
const int MAX_CONF_VALUES         = 7;
const int MAX_ARRAY_VALUES	  	  = 20;
// EEPROM
const char * const EEPROM_CONFIG  = "SYSCONFIG";
const char * const EEPROM_BOOT    = "SYSBOOT";
const int EEPROM_BANK_SIZE        = (2 * 1024);
const int EEPROM_CONFIG_SIZE      = (1536);
const int EEPROM_BOOT_SIZE        = (512);
// RESTART
const int NUM_RESTARTS_DAY        = 4;
// WATCHDOG
const bool WATCHDOG_ACTIVE = false;

#endif








