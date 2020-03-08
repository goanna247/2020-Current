#pragma once

#include "strategy/Strategy.h"
#include "strategy/StrategyBuilder.h"
#include "strategy/StrategyController.h"

// Strategy files
#include "Strategies/TurretStrategy.h"
#include "Strategies/IntakeStrategy.h"
#include "Strategies/MagLoaderStrategy.h"
#include "DriveSystem.h"


class AutoController {
 public:
  AutoController(
    Turret &turret,
    Intake &intake,
    MagLoader &magLoader,
    wml::Drivetrain &drivetrain,
    WayFinder &wayFinder,
    RobotMap &robotMap) :
      _turret(turret), 
      _intake(intake), 
      _magLoader(magLoader), 
      _drivetrain(drivetrain),
      _wayFinder(wayFinder),
      _robotMap(robotMap){
    _wayFinder.AutoConfig(ControlMap::MaxAutoDrivetrainSpeed, ControlMap::MaxAutoTurnSpeed);
  }

  void Update(double dt) {
    std::cout << "This is running but not working" << std::endl;
    // Schedule()
    // switch(_robotMap.autonomous.AutoSelecter) {
    //   case 1: // 8 ball auto 
    //     builder.Start();
    //     //fire 3 
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::way8point1));
    //     builder.Then();
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::way8point2));
    //     builder.Then();
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::way8point3));
    //     builder.Then();
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::way8point4));
    //     //shoot 5 balls 
    //     builder.Build();
    //   break;

    //   case 2: // 6 ball auto 
    //     builder.Start();
    //     //shoot 3 balls 
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::way6point1));
    //     builder.Then();
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::way6point2));
    //     builder.Then();
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::way6point3));
    //     //shoot 3 balls 
    //     builder.Build();
    //   break;

    //   case 3: // 3 ball left auto 
    //     builder.Start();
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::Lway3point1));
    //     builder.Build();
    //   break;

    //   case 4: // 3 ball middle auto 
    //     builder.Start();
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::Mway3point1));
    //     builder.Then();
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::Mway3point2));
    //     builder.Then();
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::Mway3point3));
    //     //shoot 3 balls
    //     builder.Build();
    //   break;

    //   case 5: // 3 ball right auto 
    //     builder.Start();
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::Rway3point1));
    //     builder.Then();
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::Rway3point2));
    //     builder.Then();
    //     builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _wayFinder, _robotMap, ControlMap::Rway3point3));
    //     //shoot 3 balls
    //     builder.Build();
    //   break;

    // }
  }

 private:
  Turret &_turret;
  Intake &_intake;
  MagLoader &_magLoader;
  wml::Drivetrain &_drivetrain;
  WayFinder &_wayFinder;
  RobotMap &_robotMap;


  wml::StrategyBuilder builder;
  frc::Timer Autotimer;
};
