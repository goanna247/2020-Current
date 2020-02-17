#include "BeltIntake.h"
#include <iostream>
using namespace wml;
using namespace wml::controllers;

BeltIntake::BeltIntake(Gearbox &BeltIntakeMotors, 
											 actuators::DoubleSolenoid &IntakeDown, 
											 SmartControllerGroup &contGroup,
											 bool &FlyWheelToggle,
											 bool &TurretToggle) : 
											 
											 _BeltIntakeMotors(BeltIntakeMotors), 
											 _IntakeDown(IntakeDown),  
											 _contGroup(contGroup),
											 _FlyWheelToggle(FlyWheelToggle),
											 _TurretToggle(TurretToggle) {
	_IntakeDown.SetTarget(wml::actuators::BinaryActuatorState::kReverse); // Default state
}

void BeltIntake::TeleopOnUpdate(double dt) {


	if (_contGroup.Get(ControlMap::DownIntake, Controller::ONRISE)) {
		if (ToggleEnabled) {
			_IntakeDown.SetTarget(wml::actuators::BinaryActuatorState::kForward);
			_FlyWheelToggle = false;
			ToggleEnabled = false;
		} else if (!ToggleEnabled) {
			_IntakeDown.SetTarget(wml::actuators::BinaryActuatorState::kReverse);
			_FlyWheelToggle = true;
			ToggleEnabled = true;
		}
	} 

	if (_FlyWheelToggle) {
		IntakePower = _contGroup.Get(ControlMap::Intake) > ControlMap::triggerDeadzone ? _contGroup.Get(ControlMap::Intake) :
		_contGroup.Get(ControlMap::Outake) > ControlMap::triggerDeadzone ? -_contGroup.Get(ControlMap::Outake) : 0;
	}	else {
		IntakePower = 0;
	}

	_IntakeDown.Update(dt);
	_BeltIntakeMotors.transmission->SetVoltage(12 * IntakePower);
}

void BeltIntake::AutoOnUpdate(double dt) {}

void BeltIntake::TestOnUpdate(double dt) {

	timer.Start();

	switch (TestType) {
		case 1: // forwards
			if (timer.Get() < timeout) {
				_IntakeDown.SetTarget(wml::actuators::kForward);
				IntakePower = 1;
			} else {
				IntakePower = 0;
				TestType++;
				timer.Reset();
			}
		break;

		case 2:
			if (timer.Get() < timeout) {
				IntakePower = -1;
			} else {
				timer.Reset();
				TestType++;
				_IntakeDown.SetTarget(wml::actuators::kReverse);
			}
		break;
	}
	_BeltIntakeMotors.transmission->SetVoltage(12 * IntakePower);
	_IntakeDown.Update(dt);
}
