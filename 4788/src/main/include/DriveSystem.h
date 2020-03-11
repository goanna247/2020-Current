#pragma once 

#include "controllers/Controllers.h"
#include "strategy/Strategy.h"
#include "RobotMap.h"

// WayFinder
#include "wayfinder.h"


// Class that Runs In Manual Drive e.g Human Drivers
class DrivetrainManual : public wml::Strategy {
  public:
    DrivetrainManual(std::string name, 
                     wml::Drivetrain &drivetrain,
                     wml::actuators::DoubleSolenoid &ChangeGears, 
                     wml::actuators::DoubleSolenoid &Shift2PTO, 
                     frc::Servo &PTORatchetLeft,
                     frc::Servo &PTORatchetRight,
                     wml::controllers::SmartControllerGroup &contGroup);

    void OnUpdate(double dt) override;

  private:
    wml::Drivetrain &_drivetrain;
    wml::actuators::BinaryActuator &_ChangeGears;
    wml::actuators::BinaryActuator &_Shift2PTO;
    frc::Servo &_PTORatchetLeft;
    frc::Servo &_PTORatchetRight;
    wml::controllers::SmartControllerGroup &_contGroup;
    double leftSpeed = 0, rightSpeed = 0;
    double currentSpeed;
    bool PTOactive = false;

    // NT
    nt::NetworkTableEntry LPower;
    nt::NetworkTableEntry RPower;
    nt::NetworkTableEntry LEC;
    nt::NetworkTableEntry REC;
};

// Class that Runs in Autonomous
  // class DrivetrainAuto : public wml::Strategy {
  //   public:
  //     DrivetrainAuto(
//        wml::Drivetrain &drivetrain,
//        RobotMap &robotMap,
//        WayFinder &wayfinder):
//         _drivetrain(drivetrain),
//         _robotMap(robotMap),
//         _wayfinder(wayfinder
// ){
//        Requires(&drivetrain);
//        SetCanBeInterrupted(true);
//        SetCanBeReused(true);
//        _wayfinder.AutoConfig(ControlMap::MaxAutoDrivetrainSpeed, ControlMap::MaxAutoTurnSpeed);
      //  }


//     // void OnUpdate(double dt) override {
//     //   if (_wayFinder.GetWayPointComplete()) {
//     //     IsFinished();
//     //   } else {
//     //     // _wayFinder.GotoWaypoint(1, 1, 0, 1, 1, 0, false, dt);
//     //   }
//     // }

//       //auto code 
//    }

  //  private:
//     wml::Drivetrain &_drivetrain;
//     RobotMap &_robotMap;
//     WayFinder &_wayfinder;
//     double LeftPower = 0, RightPower = 0;
//     double currentSpeed;

//     double DistanceInRotations;
//     double TurnPreviousError;
//     double TurnSum;
//     double CurrentHeading;
 // };

  // class DriveTrainAuto : public wml::Strategy {
  //   public:
  //     DriveTrainAuto(
  //       wml::Drivetrain &drivetrain,
  //       WayFinder &wayFinder,
  //       RobotMap &robotMap):
  //       _drivetrain(drivetrain),
  //       _wayFinder()
  //       {

  //     }

  //   private:

  // };

// Class that Runs in Test Mode
class DrivetrainTest : public wml::Strategy {
  public:
    DrivetrainTest(wml::Drivetrain &drivetrain, 
                   wml::control::PIDGains gains);

    void OnUpdate(double dt) override;
  private:
    wml::Drivetrain &_drivetrain;
    wml::control::PIDController _pid;
    double leftSpeed = 0, rightSpeed = 0;
    int testSelect = 1;
};