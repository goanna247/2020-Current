#pragma once

#include "strategy/StrategySystem.h"
#include "RobotMap.h"
#include "PIDScheduledController.h"

using actState = wml::actuators::BinaryActuatorState;

enum class TurretRotationState {
  IDLE,
  ZEROING,
  MANUAL,
  PID
};

enum class TurretAngleState {
  IDLE,
  ZEROING,
  MANUAL,
  PID
};

enum class TurretFlywheelState {
  IDLE,
  MANUAL,
  AUTO
};

class Turret : public wml::StrategySystem {
 public:
  Turret(wml::Gearbox &turretRotationGearbox,
         wml::Gearbox &turretAngleGearbox,
         wml::Gearbox &flywheelGearbox,
         wml::sensors::LimitSwitch &rotZeroSensor,
         wml::sensors::LimitSwitch &angleZeroSensor,
         PIDGains &RotationPID,
         PIDGains &AnglePID) :
         _turretRotationGearbox(turretRotationGearbox),
         _turretAngleGearbox(turretAngleGearbox),
         _flywheelGearbox(flywheelGearbox),
         _rotZeroSensor(rotZeroSensor),
         _angleZeroSensor(angleZeroSensor),
         RotationPID{RotationPID, 0},
         AnglePID{AnglePID, 0} {
           auto inst = nt::NetworkTableInstance::GetDefault();
          _visionTable = inst.GetTable("VisionTracking");
          _table = _visionTable->GetSubTable("Target");

          imageHeight = _table->GetNumber("ImageHeight", 0); 
          imageWidth = _table->GetNumber("ImageWidth", 0); 
         } 

  // Schedule Gains
  double ScheduleGains(double dt) {
    if (abs(targetX) < (abs(imageWidth)/8)) {
      RotationPID.SetGains(RotationPID3);
      AnglePID.SetGains(AnglePID3);
      dt = 0.5; // Make accumulator awesome baby
    } else if (abs(targetX) < (abs(imageWidth)/6)) {
      RotationPID.SetGains(RotationPID2);
      AnglePID.SetGains(AnglePID2);
    } else {
      RotationPID.SetGains(RotationPID1);
      AnglePID.SetGains(AnglePID1);
    }
    return dt;
  }

  double CalculateAngleSetpoint(double ty) {
    // Setpoint 1
		double Yvalue1 = 85;
		double ECvalue1 = 0.09;
		// Setpoint 2
		double Yvalue2 = 225;
		double ECvalue2 = 0.121;

	  // Calculate goal
    double EC = ECvalue2 - ECvalue1;
    double YV = Yvalue2 - Yvalue1;
    double Gradient = (EC/YV);

    double Intercept = (ECvalue1 - (Gradient * Yvalue1));
    double Goal = ((Gradient * ty) + Intercept);

    return Goal;
  }
  
  // Set the Turrets Subsystems
  void SetTurretRotation(const TurretRotationState st, double setpoint) {
    // Only reset on a state change (updates to setpoint are continuous for the turret)
    if (st == TurretRotationState::PID)
      RotationPID.SetSetpoint(setpoint, _turretRotationState != TurretRotationState::PID);
    _turretRotationState = st;
    _rotationSetpoint = setpoint;
  }

  void SetTurretAngle(const TurretAngleState st, double setpoint) {
    // Only reset on a state change (updates to setpoint are continuous for the turret)
    if (st == TurretAngleState::PID)
      AnglePID.SetSetpoint(setpoint, _turretAngleState != TurretAngleState::PID);
    _turretAngleState = st;
    _angleSetpoint = setpoint;
  }

  void SetTurretFlywheel(const TurretFlywheelState st, double setpoint) {
    _turretFlywheelState = st;
    _flywheelSetpoint = setpoint;
  }

  void UpdateTurretRotation(double dt) {
    double voltage = 0;
    switch (_turretRotationState) {
      case TurretRotationState::IDLE:
        voltage = 0;
       break;

      case TurretRotationState::MANUAL:
        voltage = 12 * _rotationSetpoint;
       break;
      
      case TurretRotationState::PID:
        voltage = -RotationPID.Calculate(_turretRotationGearbox.encoder->GetEncoderRotations(), dt, 0.0);
        voltage *= ControlMap::MaxTurretSpeed;
       break;
      
      case TurretRotationState::ZEROING:
        if (!_rotZeroSensor.Get()) {
          voltage = 12 * 0.12;
        } else {
          _turretRotationGearbox.encoder->ZeroEncoder();
          TurretZeroed = true;
          SetTurretRotation(TurretRotationState::IDLE, 0);
        }
       break;
    }
    _turretRotationGearbox.transmission->SetVoltage(voltage);
  }


  void UpdateTurretAngle(double dt) {
    double voltage = 0;
    switch (_turretAngleState) {
      case TurretAngleState::IDLE:
        voltage = 0;
       break;

      case TurretAngleState::MANUAL:
        voltage = 12 * _angleSetpoint;
       break;

      case TurretAngleState::PID:
        voltage = AnglePID.Calculate(_turretAngleGearbox.encoder->GetEncoderRotations(), dt, 0.0);
        voltage *= ControlMap::MaxTurretAngularSpeed;
       break;

      case TurretAngleState::ZEROING:
        if (!_angleZeroSensor.Get()) {
          voltage = 12 * -0.2;
        } else {
          _turretAngleGearbox.encoder->ZeroEncoder();
          AngleZeroed = true;
          SetTurretAngle(TurretAngleState::IDLE, 0);
        }
       break;
    }
    _turretAngleGearbox.transmission->SetVoltage(voltage);
  }

  void UpdateTurretFlywheel(double dt) {
    double voltage = 0;
    switch (_turretFlywheelState) {
      case TurretFlywheelState::IDLE:
        voltage = 0;
       break;

      case TurretFlywheelState::MANUAL:
        voltage = _flywheelSetpoint;
       break;

      case TurretFlywheelState::AUTO:
        voltage = _flywheelSetpoint;
       break;
    }
    _flywheelGearbox.transmission->SetVoltage(voltage);
  }

  void Update(double dt) {
    targetX = _table->GetNumber("Target_X", 0);
    targetY = _table->GetNumber("Target_Y", 0);
    dt = ScheduleGains(dt);
    UpdateTurretRotation(dt);
    UpdateTurretAngle(dt);
    UpdateTurretFlywheel(dt);
  }

  // Gearboxes
  wml::Gearbox &_turretRotationGearbox, &_turretAngleGearbox, &_flywheelGearbox;

  // Sensors
  wml::sensors::LimitSwitch &_rotZeroSensor, &_angleZeroSensor;

  bool TurretZeroed = false;
  bool AngleZeroed = false;

  // PID
  PIDScheduledController RotationPID;
  PIDScheduledController AnglePID;

  double targetX;
  double targetY;
  double imageHeight;
  double imageWidth;

 private:
  // States
  TurretRotationState _turretRotationState{TurretRotationState::IDLE};
  TurretAngleState _turretAngleState{TurretAngleState::IDLE};
  TurretFlywheelState _turretFlywheelState{TurretFlywheelState::IDLE};


  PIDGains RotationPID1{"Turret General Gains", 0.05, 0.0, 0.001};
  PIDGains AnglePID1{"Angle General Gains", 15.0, 5.0, 0.0};

  // Scheduled gains
  PIDGains RotationPID2{"Roation Precise Gains", 0.07, 0.001, 0.001};
  PIDGains AnglePID2{"Angle Precise Gains", 15.0, 5.0, 0.0};
  
  PIDGains RotationPID3{"Rotation Lock On", 0.1, 0.003, 0.001};
  PIDGains AnglePID3{"Angle Lock On", 15.0, 5.0, 0.0};

  // NT
  std::shared_ptr<nt::NetworkTable>_visionTable;
  std::shared_ptr<nt::NetworkTable>_table;

  // Setpoints
  double _rotationSetpoint = 0, _angleSetpoint = 0, _flywheelSetpoint = 0;
};