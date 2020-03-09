#pragma once 

#include "strategy/StrategySystem.h"
#include "RobotMap.h"

using actState = wml::actuators::BinaryActuatorState;

enum class IntakeState {
  STOWED, // Up, motor off
  DEPLOYED, // Down
};

class Intake : public wml::StrategySystem {
 public:
  Intake(wml::Gearbox &intakeGearbox,
         wml::actuators::DoubleSolenoid &intakeActuator) :
         _intakeGearbox(intakeGearbox), _intakeActuator(intakeActuator) {}

  void SetIntake(const IntakeState st, double setpoint) {
    _intakeState = st;
    _intakeSetpoint = setpoint;
  }

  void UpdateIntake(double dt) {
    double voltage = 0;
    switch (_intakeState) {
      case IntakeState::STOWED:
        voltage = 0;
        _intakeActuator.SetTarget(actState::kReverse);
       break;
      
      case IntakeState::DEPLOYED:
        voltage = 12 * _intakeSetpoint;
        _intakeActuator.SetTarget(actState::kForward);
       break;
    }
    _intakeGearbox.transmission->SetVoltage(voltage);
    _intakeActuator.Update(dt);
  }

  void Update(double dt) {
    UpdateIntake(dt);
  }

 private:
  // Gearboxes
  wml::Gearbox &_intakeGearbox;
  wml::actuators::DoubleSolenoid &_intakeActuator;

  // States
  IntakeState _intakeState{IntakeState::STOWED};

  // Setpoints
  double _intakeSetpoint;
};