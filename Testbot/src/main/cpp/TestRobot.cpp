#include "TestRobot.h"
#include <iostream>

#include "frc/AnalogInput.h"
#include <actuators/VoltageController.h>
#include "sensors/LimitSwitch.h"

#include <math.h>

void Robot::RobotInit() {
	xbox1 = new wml::controllers::XboxController(0);
}

void Robot::AutonomousInit() {}
void Robot::AutonomousPeriodic() {}

void Robot::TeleopInit() {}
void Robot::TeleopPeriodic() {

	

	if (xbox1->GetButton(wml::controllers::XboxController::kA)) {
		std::cout << "Dpad Works!" << std::endl;
	}
}
void Robot::TestInit() {}
void Robot::TestPeriodic() {}
