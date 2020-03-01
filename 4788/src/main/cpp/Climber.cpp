#include "Climber.h"
#include <iostream>

using namespace wml;
using namespace wml::controllers;

Climber::Climber(actuators::DoubleSolenoid &ClimberActuator,
                 actuators::DoubleSolenoid &BeltActuator,
                 Gearbox &ClimberElevatorLeft, 
                 Gearbox &ClimberElevatorRight,
                 SmartControllerGroup &contGroup,
                 bool &TurretDisable) : 

                 _ClimberActuator(ClimberActuator), 
                 _BeltActuator(BeltActuator),
                 _ClimberElevatorLeft(ClimberElevatorLeft), 
                 _ClimberElevatorRight(ClimberElevatorRight),
                 _contGroup(contGroup),
                 _TurretDisable(TurretDisable) {

}


void Climber::TeleopOnUpdate(double dt) {
  if (_contGroup.Get(ControlMap::ClimberToggle, Controller::ONRISE)) {

    if (ToggleEnabled) {
      ToggleEnabled = false;
      _TurretDisable = false;
    } else if (!ToggleEnabled) {
      ClimberTimer.Start();
      ToggleEnabled = true;
      _TurretDisable = true;
    }
  } 
 
  if (ToggleEnabled) {

    // Put Intake down if up
    _BeltActuator.SetTarget(wml::actuators::BinaryActuatorState::kForward);

    if (ClimberTimer.Get() > 1) {
      _ClimberActuator.SetTarget(wml::actuators::BinaryActuatorState::kReverse);
      ClimberTimer.Reset();
      ClimberTimer.Stop();
    }

    liftSpeedleft = abs(_contGroup.Get(ControlMap::ClimberControlLeft)) > ControlMap::joyDeadzone ?  _contGroup.Get(ControlMap::ClimberControlLeft) : 0;
    liftSpeedright = abs(_contGroup.Get(ControlMap::ClimberControlRight)) > ControlMap::joyDeadzone ? _contGroup.Get(ControlMap::ClimberControlRight) : 0;
    liftSpeedright *= ControlMap::LiftMaxSpeed;
    liftSpeedleft *= ControlMap::LiftMaxSpeed;
  } else {
    _ClimberActuator.SetTarget(wml::actuators::BinaryActuatorState::kForward);
  }

  _ClimberElevatorLeft.transmission->SetVoltage(12 * liftSpeedleft);
  _ClimberElevatorRight.transmission->SetVoltage(12 * -liftSpeedright);
  _ClimberActuator.Update(dt);
  _BeltActuator.Update(dt);
}

void Climber::AutoOnUpdate(double dt) {}

void Climber::TestOnUpdate(double dt) {

  switch (testType) {
    case 1:
      if (_ClimberElevatorLeft.encoder->GetEncoderRotations() <= 6) {
        _ClimberActuator.SetTarget(wml::actuators::kForward);
        liftSpeedleft = 0.5;
        liftSpeedright = 0.5;
      } else {
        liftSpeedleft = 0;
        liftSpeedright = 0;
        testType++;
      }
    break;

    case 2:
      if (_ClimberElevatorLeft.encoder->GetEncoderRotations() >= 6) {
        _ClimberActuator.SetTarget(wml::actuators::kForward);
        liftSpeedleft = -0.5;
        liftSpeedright = -0.5;
      } else if (_ClimberElevatorLeft.encoder->GetEncoderRotations() <= 0) {
        liftSpeedleft = 0;
        liftSpeedright = 0;
        testType++;
      }
    break;

    case 3:
      _ClimberActuator.SetTarget(wml::actuators::kReverse);
      testType++;
    break;
  }

  _ClimberElevatorLeft.transmission->SetVoltage(liftSpeedleft);
  _ClimberElevatorRight.transmission->SetVoltage(liftSpeedright);
  _ClimberActuator.Update(dt);
}
