#pragma once 

#include "strategy/StrategySystem.h"
#include "RobotMap.h"

enum class IntakeDownState { 
  STOWED,
  DEPLOYED,
  INTAKING,
  EJECTING,
  IDLE
};


class Intake : public wml::StrategySystem {
  public:
    Intake(wml::Gearbox &BeltIntakeMotors,
           wml::actuators::DoubleSolenoid &IntakeDown);

  void SetDown(const IntakeDownState st, double setpoint) {
    _downState = st;
    _downSetPoint = setpoint;
  } 



  void UpdateSetMove(double dt) {

    switch(_rotationState) {
      case IntakeMovingState::IDLE:
        RotationPower = 0;
      break;

      case IntakeMovingState::MANUAL:

      break;
    }
    _BeltIntakeMotors.transmission->SetVoltage(RotationPower);
  }

  void UpdateSetDown(double dt) {

    switch(_DownState) {
      case IntakeDownState::IDLE:

      break;

      case IntakeDownState::MANUAL:
        SetShifter(actuator::)
      break;
    }
  }

  void Update(double dt) {
    UpdateSetDown();
    UpdateSetMove();
  }


  private: 

  wml::Gearbox &_BeltIntakeMotors;
  wml::actuators::DoubleSolenoid &_IntakeDown;

  frc::Timer timer;

  /* define gear boxes, sensors, tables and whatever the fuck this is 
    TurretRotationState _rotState{TurretRotationState::MANUAL};
  TurretFlywheelState _flywheelState{TurretFlywheelState::MANUAL};
  */

 IntakeDownState _downSetPoint{IntakeDownState::STOWED};
 IntakeDownState 

  IntakeDownState _
  IntakeMovingState 

}