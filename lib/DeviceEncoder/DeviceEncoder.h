#ifndef DeviceEncoder_h
#define DeviceEncoder_h

#include <Arduino.h>
#include <SPI.h>

class DeviceEncoder {
    public:
        DeviceEncoder(int pin_cs); // Constructor con el pin CS
        bool begin(long max_pulses, float data_interval[], bool cyclic); // Inicialización del encoder
        void reset_counter(long counter_start = 0); // Restablecer el contador a un valor específico
        long get_counter(bool raw = false); // Obtener el valor actual del contador
        float get_data(); // Obtener el dato calculado a partir del contador

    private:
        int _pin_cs; // Pin de selección de chip (Chip Select) para el encoder
        long _counter_interval; // Intervalo máximo del contador
        float _data_interval[2]; // Intervalo de los datos
        bool _counter_never_ending; // Indicador de contador continuo
        float _k; // Factor de conversión del contador a datos

        long _read_counter(); // Leer el valor actual del contador del encoder
};

#endif // DeviceEncoder_h
