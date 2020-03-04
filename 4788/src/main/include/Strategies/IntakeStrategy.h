#include "controllers/Controller.h"
#include "strategy/Strategy.h"
#include "BIntake2.h"

class IntakeManualStrategy : wml::Strategy {
 public:
  IntakeManualStrategy(
    const Intake &intake,
    const wml::actuators::DoubleSolenoid &IntakeDown,
    const wml::controllers::SmartControllerGroup &controllers,
  ) : wml::Strategy("Manual"), _beltIntake(intake), _controllers(controllers) {
    Requires(&intake);
    SetCanBeInterrupted(true);
    SetCanBeReused(true);
  }

  void OnUpdate(double dt) override {
    actuators::BinaryActuatorState pistonState{actuators::BinaryActuatorState::kForward};
    double power = 0;

    switch (_state) {
      case IntakeDownState::DEPLOYED:
        pistonState = actuators::BinaryActuatorState::kForward;
      break;

      case IntakeDownState::STOWED:
        pistonState = actuators::BinaryActuatorState::kReverse;
      break;

      case IntakeDownState::INTAKING:
        power = 0.7;
      break;

      case IntakeDownState::EJECTING:
        power = -0.7;
      break;

      case IntakeDownState::IDLE:
        power = 0;
      break;
    }

    if (_contGroup.Get(ControlMap::DownIntake, Controller::ONRISE)) {
      _state = IntakeDownState::DEPLOYED
      if (_contGroup.Get(ControlMap::Intake)) {
        _state = IntakeDownState::INTAKING;
      } else {
        _state = IntakeDownState::IDLE;
      }
    } else { 
      _state = IntakeDownState::STOWED;
    }

    
  }

 private:
  const BeltIntake &_beltIntake;
  const wml::actuators::DoubleSolenoid &_IntakeDown;
  const wml::controllers::SmartControllerGroup &_controllers;
}take.h
