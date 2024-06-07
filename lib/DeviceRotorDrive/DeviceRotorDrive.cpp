/*
-------------------------------------------------------------------------------------
DeviceRotorDrive.cpp
  This file define methods for rotary motor drive.
 
 Mar 2021
 Inaki Etxebarria
-------------------------------------------------------------------------------------
*/

#include "DeviceRotorDrive.h"

DeviceRotorDrive::DeviceRotorDrive() {
	_pin_onoff = -1;
};
	

DeviceRotorDrive::DeviceRotorDrive(int pin_onoff) {
	_pin_onoff = pin_onoff;
	pinMode(_pin_onoff, OUTPUT);
	stop();
};

void DeviceRotorDrive::stop(void) {
	digitalWrite(_pin_onoff, _low_high(ROTOR_OFF));
	_onoff = ROTOR_OFF;
};

void DeviceRotorDrive::go(rotorstate state) {
	digitalWrite(_pin_onoff, _low_high(state));
	_onoff = state;
};


