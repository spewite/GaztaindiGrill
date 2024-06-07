#include "DeviceEncoder.h"

DeviceEncoder::DeviceEncoder(int pin_cs) {
    _pin_cs = pin_cs;
    pinMode(_pin_cs, OUTPUT);
    digitalWrite(_pin_cs, HIGH);
}

bool DeviceEncoder::begin(long max_pulses, float data_interval[], bool cyclic) {
    _counter_interval = max_pulses;
    _data_interval[0] = data_interval[0]; _data_interval[1] = data_interval[1];
    _counter_never_ending = cyclic;
    _k = (_data_interval[1] - _data_interval[0]) / (float)_counter_interval;

    // Configurar el encoder usando SPI.beginTransaction() y SPI.endTransaction()
    SPI.beginTransaction(SPISettings(1000000, SPI_MSBFIRST, SPI_MODE0));
    digitalWrite(_pin_cs, LOW);
    SPI.transfer(0x88); // Write to MDR0
    SPI.transfer(0x03); // Configure to 4 byte mode
    digitalWrite(_pin_cs, HIGH);
    SPI.endTransaction();
    reset_counter(0);

	return true;
}

// void DeviceEncoder::reset_counter(long counter_start) {
//     uint8_t buf[4];
//     long data = counter_start;
//     // Convierte el counter_start a bytes
//     for (int i = 3; i >= 0; i--) {
//         buf[i] = data & 0xFF;
//         data >>= 8;
//     }

//     // Restablecer el contador
//     SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//     digitalWrite(_pin_cs, LOW);
//     SPI.transfer(0x98); // Write to DTR
//     for (int i = 0; i < 4; i++) {
//         SPI.transfer(buf[i]);
//     }
//     digitalWrite(_pin_cs, HIGH);
//     SPI.endTransaction();
// }   

 void DeviceEncoder::reset_counter(long counter_start) {
    uint8_t buf[4];
    long data = counter_start;
    for (int i = 3; i >= 0; i--) {
        buf[i] = data & 0xFF;
        data >>= 8;
    }

    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
    digitalWrite(_pin_cs, LOW); // Begin SPI conversation
    SPI.transfer(0x98); // Write to DTR
    for (int i = 0; i < 4; i++) {
        SPI.transfer(buf[i]);
    }
    digitalWrite(_pin_cs, HIGH); // Terminate SPI conversation
    delayMicroseconds(100); // Provides some breathing room between SPI conversations
    digitalWrite(_pin_cs, LOW); // Begin SPI conversation
    SPI.transfer(0xE0);
    digitalWrite(_pin_cs, HIGH); // Terminate SPI conversation
    SPI.endTransaction();
}  
  
long DeviceEncoder::_read_counter(void) {
	long counter;
	uint8_t buf[4];
    SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
									// Read encoder 1
	digitalWrite(_pin_cs, LOW);     // Begin SPI conversation
	SPI.transfer(0x60); 			// Request count
	buf[3]  = SPI.transfer(0x00);   // Read highest order byte
    buf[2] = SPI.transfer(0x00);
    buf[1]  = SPI.transfer(0x00);
    buf[0]  = SPI.transfer(0x00);	// Read lowest order byte
	digitalWrite(_pin_cs, HIGH);	// Terminate SPI conversation
    SPI.endTransaction();
	//  Calculate encoder count
    counter = ((long)buf[3]<<24) + ((long)buf[2]<<16) + ((long)buf[1]<<8) + (long)buf[0];
	return counter;
};

// long DeviceEncoder::_read_counter() {
//     long counter = 0;
//     uint8_t buf[4];

//     SPI.beginTransaction(SPISettings(1000000, MSBFIRST, SPI_MODE0));
//     digitalWrite(_pin_cs, LOW);
//     SPI.transfer(0x60); // Request count
//     for (int i = 0; i < 4; i++) {
//         buf[i] = SPI.transfer(0x00);
//     }
//     digitalWrite(_pin_cs, HIGH);
//     SPI.endTransaction();

//     // Calcula el valor del contador
//     counter = ((long)buf[0] << 24) + ((long)buf[1] << 16) + ((long)buf[2] << 8) + (long)buf[3];
//     return counter;
// }

long DeviceEncoder::get_counter(bool raw) {
    long counter = _read_counter();
    if (!raw && _counter_never_ending) {
        counter %= _counter_interval;
    }
    return counter;
}

float DeviceEncoder::get_data() {
    return (_k * (float)get_counter() + _data_interval[0]);
}
