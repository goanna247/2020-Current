#pragma once

#include "controllers/Controllers.h"
#include "RobotMap.h"

//double lastTimestamp;

class BeltIntake {
  public:
    BeltIntake(wml::Gearbox &BeltIntakeMotors, wml::actuators::DoubleSolenoid &IntakeDown, wml::controllers::SmartControllerGroup &contGroup);

    void TeleopOnUpdate(double dt);
    void AutoOnUpdate(double dt);
    void TestOnUpdate(double dt, double lastTimestamp);
    frc::Timer timer;


  private:
    wml::Gearbox &_BeltIntakeMotors;
    wml::actuators::DoubleSolenoid &_IntakeDown;
    wml::controllers::SmartControllerGroup &_contGroup;
    //lastTimestamp = Timer::GetFPGATimestamp();


    bool ToggleEnabled;
};