#include "ControlPannel.h"
#include <iostream>

using namespace wml;
using namespace wml::controllers;

ControlPannel::ControlPannel(Gearbox &ControlPannelMotor, actuators::DoubleSolenoid &PannelPnSol, SmartControllerGroup &contGroup) : _ControlPannelMotor(ControlPannelMotor), _PannelPnSol(PannelPnSol), _contGroup(contGroup) {}

void ControlPannel::TeleopOnUpdate(double dt) {

	double ControlPannelPower;

	if (_contGroup.Get(ControlMap::ControlPannelUp)) {
		_PannelPnSol.SetTarget(wml::actuators::kForward);
	} else {}
	
	if (_contGroup.Get(ControlMap::ControlPannelDown)) {
		_PannelPnSol.SetTarget(wml::actuators::kReverse);
	} else {}

	if (_contGroup.Get(ControlMap::SpinControlPannelLeft)) {
		ControlPannelPower = 0.5;
	} else {}

	if (_contGroup.Get(ControlMap::SpinControlPannelRight)) {
		ControlPannelPower = -0.5;
	} else {}

	_ControlPannelMotor.transmission->SetVoltage(12 * ControlPannelPower);
}
void ControlPannel::AutoOnUpdate(double dt) {}

void ControlPannel::TestOnUpdate(double dt) {
//turning motor on for 10 seconds, forwards then reverse 
	double ControlPannelPower;
		timer.Start();
		while (timer.Get() >= 10) {
			double Speed = 1;
			_ControlPannelMotor.transmission->SetVoltage(12 * Speed);
		}
		while (timer.Get() >= 20) {
			double Speed = -1;
		_ControlPannelMotor.transmission->SetVoltage(12 * Speed);
	}
	timer.Stop();
	timer.Reset();

//actuate the solenoids 4 times 
	while ( i >= 4) {
		_PannelPnSol.SetTarget(wml::actuators::kForward);
		timer.Start();
			while (timer.Get() > 3) {}
		_PannelPnSol.SetTarget(wml::actuators::kReverse);
			while (timer.Get() > 3) {}
		timer.Stop();
		timer.Reset();
		i = i + 1;
	}
}