#pragma once

#include "strategy/Strategy.h"
#include "strategy/StrategyBuilder.h"
#include "strategy/StrategyController.h"

// Strategy files
#include "Strategies/TurretStrategy.h"
#include "Strategies/IntakeStrategy.h"
#include "Strategies/MagLoaderStrategy.h"
#include "DriveSystem.h"
#include "turretStrategy.h"




 class AutoController {
  public:
   AutoController(
    Intake &intake,
    Turret &turret,
    MagLoader &magLoader,
    wml::Drivetrain &drivetrain,
    WayFinder &wayFinder,
    RobotMap &robotMap ):
    _intake(intake),
    _turret(turret),
    _magLoader(magLoader),
    _drivetrain(drivetrain),
    _wayFinder(wayFinder),
    _robotMap(robotMap) {
    _wayFinder.AutoConfig(ControlMap::MaxAutoDrivetrainSpeed, ControlMap::MaxAutoTurnSpeed);
  }

  void Update(double dt) {
    builder.Start();

    //builder.Add(std::make_shared<DrivetrainAuto>(_drivetrain, _robotMap.driveSystem.ChangeGearing, _robotMap.driveSystem.Shift2PTO, _wayFinder, WayFinder::Waypoint{3.2, -2.4, -30, 5.4, -0.7, 30, false}));
  }

  private:
   Intake &_intake;
   Turret &_turret;
   MagLoader &_magLoader;
   wml::Drivetrain &_drivetrain;
   WayFinder &_wayFinder;
   RobotMap &_robotMap;
   wml::StrategyBuilder builder;

   frc::Timer timer;
 };
