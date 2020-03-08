#pragma once

#include <vector>
#include "PIDScheduledController.h"
#include "wayfinder.h"

#include "controllers/Controllers.h"

#define __CONTROLMAP_USING_JOYSTICK__ false

struct ControlMap {
  static void InitSmartControllerGroup(wml::controllers::SmartControllerGroup &contGroup);

  /**
   * CAN Port Number System We are using (PWM Not Included)
   * - 0-9 (Control System, e.g Pneumatics, PDP...)
   * - 10-19 (SparkMax/Brushless Motor Controllers)
   * - 20- (Regular Motor Controllers)
  **/

  // ---------------- Defined Ports ------------------

  // Controllers
  static const int XboxController1Port, XboxController2Port;
  static const int JoyController1Port, JoyController2Port, JoyController3Port;

  static const int Driver = 1;
  static const int CoDriver = 2;
  static const int DevController = 3;
  static const int override = 4;

  static const double joyDeadzone;
  static const double xboxDeadzone;
  static const double triggerDeadzone;

  static inline double doJoyDeadzone(double val) {
    return std::fabs(val) > joyDeadzone ? val : 0;
  }


  // PCM1
  static const int PCModule;


  // Drive System
  static const int DriveMAXportFL, DriveMAXportFR, DriveMAXportBL, DriveMAXportBR;
  static const int ChangeGearPort1, ChangeGearPort2;
  static const int Shift2PTOPort1, Shift2PTOPort2;
  static const double ChangeGearTime;
  static const int PTORatchetLeftPort, PTORatchetRightPort;
  static const double PTORatchetLeftPosition, PTORatchetRightPosition;
  static const double MaxDrivetrainSpeed;
  static const double MaxDrivetrainAcceleration;
  static const double DriveTestCaseRotations;
  static const double PTORatchetLeftPositionInit, PTORatchetRightPositionInit;

  // Turret
  static const bool TuneTurretPID;
  static const bool TuneAnglePID;
  static const int TurretFlyWheelPort, TurretRotationPort, TurretAnglePort, TurretFlyWheelPort2;
  static const int AngleEncoderChannelA, AngleEncoderChannelB;
  static const int TurretLeftLimitPort, TurretRightLimitPort, TurretAngleDownLimitPort;
  static const bool TurretLeftLimitInvert, TurretRightLimitInvert, TurretAngleDownLimitInvert;
  static const double TurretDistanceSetpoint1, TurretDistanceSetpoint2, TurretDistanceSetpoint3;
  static const double TurretZeroTimeoutSeconds;
  static const double TurretEncoderSafeZone;
  static const double MaxTurretSpeed, MaxTurretAngularSpeed, FlyWheelVelocity;
  static const double TurretEncoderRotations;
  static const double MaxAngleEncoderRotations;
  static const double TurretRatio, TurretGearBoxRatio;
  static const int FlyWheelEncoderPort1, FlyWheelEncoderPort2;
  static const int AngleEncoderPort1, AngleEncoderPort2;

  // Y Axis Setpoints
  static const double AngleSetPoint[100];
  static const double AngleSetpoint1;
  static const double AngleSetpoint2;
  static const double AngleSetpoint3;
  static const double AngleSetpoint4;
  static const double AngleSetpoint5;
  static const double AngleSetpoint6;
  static const double AngleSetpoint7;
  static const double AngleSetpoint8;
  static const double AngleSetpoint9;
  static const double AngleSetpoint10;

  // Intake
  static const int IntakeMotorPort;
  static const int IntakeDownPort1, IntakeDownPort2;
  static const double PannelActuationTime;
  static const double IntakeDownActuationTime;
  static const double IntakeTestCaseRotations;

  // MagLoader
  static const int MagLoaderMotorPort;
  static const int StartMagLimitPort, Position1LimitPort, Position5LimitPort;
  static const double MagazineBallThreshStart;
  static const double MagazineBallThreshFinal;
  static const double MagazineBallThreshIndex;
  static const double MagTestCaseRotations;

  //Control Pannel
  static const int ControlPannelPort;
  static const int ControlPannelUpPort;
  static const int ControlPannelUpSolPort1;
  static const int ControlPannelUpSolPort2;
  static const double ControlPannelActuationTime;
  static const bool ControlClimb;

  // Climber
  static const int ClimberActuatorPort1, ClimberActuatorPort2;
  static const double ClimberActuationTime;
  static const int Shift1PTOPort, Shift2PTOPort;
  static const int ClimberMotor1Port, ClimberMotor2Port;
  static const double LiftMaxSpeed;
  static const double ShiftPTOActuationTime;


  // Control System
  static const int PressureSensorPort;
  static const int CompressorPort;
  static const int CamFOV;


  // Auto Values
  static const double AutoGearRatio; // 1:AutoGearRatio
  static const double WheelDiameter; // CM
  static const double WheelCircumference;

  static const double MaxAutoDrivetrainSpeed, MaxAutoTurnSpeed;
  


  // Drive PID
  static const double DriveKp, DriveKi, DriveKd;

  //waypoints 
  static const WayFinder::Waypoint way8point1;
  static const WayFinder::Waypoint way8point2;
  static const WayFinder::Waypoint way8point3;
  static const WayFinder::Waypoint way8point4;

  static const WayFinder::Waypoint way6point1;
  static const WayFinder::Waypoint way6point2;
  static const WayFinder::Waypoint way6point3;

  static const WayFinder::Waypoint Lway3point1;


  static const WayFinder::Waypoint Mway3point1;
  static const WayFinder::Waypoint Mway3point2;
  static const WayFinder::Waypoint Mway3point3;

  static const WayFinder::Waypoint Rway3point1;
  static const WayFinder::Waypoint Rway3point2;
  static const WayFinder::Waypoint Rway3point3;



  //Turret PID
  // static const PIDGains TurretRotationPID, TurretAnglePID;

  // --------------- Defined Buttons -----------------

  // Turret PID Tuner
  static const wml::controllers::tButton kpUP, kpDOWN;
  static const wml::controllers::tButton kiUP, kiDOWN;
  static const wml::controllers::tButton kdUP, kdDOWN;

  // Drivetrain
  #if __CONTROLMAP_USING_JOYSTICK__
  static const wml::controllers::tAxis DrivetrainForward, DrivetrainTurn;
  #else
  static const wml::controllers::tAxis DrivetrainLeft, DrivetrainRight;
  #endif
  static const wml::controllers::tButton ReverseDrivetrain;
  static const wml::controllers::tButton ShiftGears;
  static const wml::controllers::tButton Defence;
  static const wml::controllers::tButton Servo;

  // Turret
  #if __CONTROLMAP_USING_JOYSTICK__
  //@TODO
  #else
  static const wml::controllers::tAxis TurretAutoAimAxis;
  static const std::vector<wml::controllers::tButton> TurretAutoAim;

  static const wml::controllers::tAxis TurretManualRotate;
  static const wml::controllers::tAxis TurretManualAngle;
  static const wml::controllers::tAxis TurretFlyWheelSpinUp;
  static const wml::controllers::tButton TurretFire;
  static const wml::controllers::tButton Ball3Fire; // just for auto testing 
  static const wml::controllers::tButton RevFlyWheel;
  #endif


  // Intake
  #if __CONTROLMAP_USING_JOYSTICK__

  #else
  static const wml::controllers::tAxis Intake;
  static const wml::controllers::tAxis Outake;
  static const std::vector<wml::controllers::tButton> DownIntake;
  #endif

  //Control Pannel
  #if __CONTROLMAP_USING_JOYSTICK__

  #else 
  static const wml::controllers::tPOV ControlPannelUp;
  static const wml::controllers::tPOV SpinControlPannelLeft;
  static const wml::controllers::tPOV SpinControlPannelRight;
  static const wml::controllers::tPOV ControlPannelDown;

  #endif

  // MagLoader
  static const wml::controllers::tPOV ShiftMagazinePOV;

  static const wml::controllers::tButton ShiftUpMagazine;
  static const wml::controllers::tButton ShiftDownMagazine;
  static const wml::controllers::tButton ManualMag;
  //Climber
  #if __CONTROLMAP_USING_JOYSTICK__

  #else
  static const wml::controllers::tAxis ClimberControlRight;
  static const wml::controllers::tButton Shift2PTO; // Toggle
  static const wml::controllers::tAxis ClimberControlLeft;
  static const wml::controllers::tButton ClimberToggle;
  #endif

};