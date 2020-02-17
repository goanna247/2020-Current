#pragma once

#include "controllers/Controllers.h"
#include "RobotMap.h"

class BeltIntake {
  public:
    BeltIntake(wml::Gearbox &BeltIntakeMotors, 
               wml::actuators::DoubleSolenoid &IntakeDown, 
               wml::controllers::SmartControllerGroup &contGroup,
               bool &FlyWheelToggle,
               bool &TurretToggle);
    
    void TeleopOnUpdate(double dt);
    void AutoOnUpdate(double dt);
    void TestOnUpdate(double dt); 

  private:
    wml::Gearbox &_BeltIntakeMotors;
    wml::actuators::DoubleSolenoid &_IntakeDown;
    wml::controllers::SmartControllerGroup &_contGroup;
    bool &_FlyWheelToggle;
    bool &_TurretToggle;


    frc::Timer timer;
    double IntakePower = 0;
    int TestType = 1;
    int timeout = 3;

    bool ToggleEnabled = false;
    bool ToggleIntakeOn = false;
};