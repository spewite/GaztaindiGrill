/*
-------------------------------------------------------------------------------------
DeviceRotorDrive.h
  This Header define api for rotary motor drive.
 
 Mar 2021
 Inaki Etxebarria
-------------------------------------------------------------------------------------
*/

#include <WiFi.h>

enum rotorstate {ROTOR_ON=1, ROTOR_OFF=0};

class DeviceRotorDrive {
	private:
		int _pin_onoff;
		rotorstate _onoff;
		inline bool _low_high(rotorstate onoff) const { return (onoff == ROTOR_ON ? HIGH : LOW); };
	public:
		DeviceRotorDrive();
		DeviceRotorDrive(int);
		void stop(void);
		void go(rotorstate);
		inline rotorstate get_state(void) const { return _onoff; };
};



