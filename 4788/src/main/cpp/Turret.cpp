#include "Turret.h"
#include <iostream>

using namespace wml;
using namespace wml::controllers;

Turret::Turret(Gearbox &Rotation,
 							 Gearbox &VerticalAxis, 
							 Gearbox &FlyWheel, 
							 sensors::BinarySensor &LeftLimit, 
							 sensors::BinarySensor &AngleDownLimit, 
							 SmartControllerGroup &contGroup, 
							 std::shared_ptr<nt::NetworkTable> &visionTable,
							 std::shared_ptr<nt::NetworkTable> &rotationTable,
							 bool &FlyWheelToggle, 
							 bool &TurretToggle,
							 int &autoSelector,
							 bool &StartDoComplete,
							 bool &strt,
							 bool &p1,
							 bool &p2,
							 bool &p3,
							 bool &end): 
							 
							 _RotationalAxis(Rotation),
						   _VerticalAxis(VerticalAxis), 
							 _FlyWheel(FlyWheel), 
							 _LeftLimit(LeftLimit), 
							 _AngleDownLimit(AngleDownLimit), 
							 _contGroup(contGroup), 
							 _visionTable(visionTable),
							 _rotationTable(rotationTable),
							 _FlyWheelToggle(FlyWheelToggle),
							 _TurretToggle(TurretToggle),
							 _autoSelector(autoSelector),
							 _StartDoComplete(StartDoComplete),
							 _strt(strt),
							 _p1(p1),
							 _p2(p2),
							 _p3(p3),
							 _end(end){
	table = _visionTable->GetSubTable("Target");
	table_2 =  _rotationTable->GetSubTable("turretRotation");

	imageHeight = table->GetNumber("ImageHeight", 0); 
	imageWidth = table->GetNumber("ImageWidth", 0);
}

void Turret::ZeroTurret() {
	// Zero Encoder on left limit
	ZeroTimer.Start();

	// Get Left Limit
	Turret::TurretZeroLeft(ZeroTimer.Get());

	// Minimum Rotations
	MinRotation = (_RotationalAxis.encoder->GetEncoderRotations() + ControlMap::TurretEncoderSafeZone);
	
	// Get Right Limit
//	Turret::TurretZeroRight(ZeroTimer.Get());

	// Get Max Rotations
	MaxRotation = (_RotationalAxis.encoder->GetEncoderTicks() - ControlMap::TurretEncoderSafeZone);

	// Get Angle Limit
	Turret::TurretZeroAngle(ZeroTimer.Get());
	
	// Reset Timers
	ZeroTimer.Stop();
	ZeroTimer.Reset();

	// Maxed Vertical Axis & Zero Flywheel
	MaxAngleRotations = (_VerticalAxis.encoder->GetEncoderRotations() + ControlMap::TurretEncoderSafeZone);
	_FlyWheel.encoder->ZeroEncoder();
}

void Turret::TeleopOnUpdate(double dt) {
	cameraSyncTimer.Start();

	targetX = table->GetNumber("Target_X", 0);
	targetY = table->GetNumber("Target_Y", 0);

	imageHeight = table->GetNumber("ImageHeight", 0); 
	imageWidth = table->GetNumber("ImageWidth", 0);

	// Tune Turret PID (If active)
	PIDTuner();


	//annas stuff dont touch 
	if (_contGroup.Get(ControlMap::Ball3Fire)) {	
		while (something) {
			AutoAimToFire(dt);
			timer.Start();
			if (ReadyToFire && timer.Get() <= Ball3Shoot) {
				_p2 = true;
			} else {
				_p2 = false;
			}
		}
		std::cout << "autoooooooooo" << std::endl;
	}	
	//annas stuff dont touch ^



	if (_contGroup.Get(ControlMap::RevFlyWheel, Controller::ONRISE)) {
		bool GetFlyWheel = _FlyWheel.transmission->GetInverted();
		RevFlywheelEntry = table->GetEntry("RevFlywheel");
		RevFlywheelEntry.SetBoolean(GetFlyWheel);
		_FlyWheel.transmission->SetInverted(!GetFlyWheel);
	}
	

	if (!_TurretToggle) {
		if (_contGroup.Get(ControlMap::TurretAutoAim)) {
			if (targetX > imageWidth || targetY > imageHeight) {
				std::cout << "Error: Target is artifacting" << std::endl;
			} else {
				RotationPower = XAutoAimCalc(dt, targetX);
				AngularPower = YAutoAimCalc(dt, targetY);
			}
		} else {
			Rsum = 0;
		}


		// Manual Angle Control
		AngularPower += std::fabs(_contGroup.Get(ControlMap::TurretManualAngle)) > ControlMap::joyDeadzone ? -_contGroup.Get(ControlMap::TurretManualAngle) : 0;

		// Manual Rotation Control
		RotationPower += std::fabs(_contGroup.Get(ControlMap::TurretManualRotate)) > ControlMap::joyDeadzone ? _contGroup.Get(ControlMap::TurretManualRotate) : 0;
		RotationPower = -(fabs(RotationPower) * RotationPower);
	} 

	if (!_FlyWheelToggle) {
		// FlyWheel Code
		if ((_contGroup.Get(ControlMap::TurretAutoAim) > ControlMap::triggerDeadzone) && (_contGroup.Get(ControlMap::TurretFlyWheelSpinUp) > ControlMap::triggerDeadzone)) {
			FlyWheelAutoSpinup();
		} else if (_contGroup.Get(ControlMap::TurretFlyWheelSpinUp) > ControlMap::triggerDeadzone) {
			FlyWheelManualSpinup();
		} else {
			FlyWheelPower = 0;
		}
	}

	// FlyWheelPower = _contGroup.Get(ControlMap::TurretFlyWheelSpinUp);

	

	// Flywheel Feedback
	ContFlywheelFeedback();


	// Limits Turret Speed
	RotationPower *= ControlMap::MaxTurretSpeed; 
	AngularPower *= ControlMap::MaxTurretAngularSpeed;

	table_2->PutNumber("Turret_Min", MinRotation);
	table_2->PutNumber("Turret_Max", MaxRotation);

	_RotationalAxis.transmission->SetVoltage(12 * RotationPower);
	_VerticalAxis.transmission->SetVoltage(12 * AngularPower);
	_FlyWheel.transmission->SetVoltage(12 * FlyWheelPower);
}

void Turret::AutoOnUpdate(double dt) {
	double targetX = table->GetNumber("Target_X", 0);
	double targetY = table->GetNumber("Target_Y", 0);
	switch (TurretAutoSelection) {
		case 1:
			switch(_autoSelector){
				case 1: // 8 ball auto, shoots 3 balls then 5 balls 
						if (!_StartDoComplete) {	
							timer.Start();
							if (timer.Get() <= Ball3Shoot){
								AutoAimToFire(dt);
								if (ReadyToFire) {
									_p2 = true;
								}
								_StartDoComplete = true;
								timer.Stop();
								timer.Reset();
						}	

						if (_strt) {
							timer.Start();
							if (timer.Get() <= Ball5Shoot){
							AutoAimToFire(dt);
								if (ReadyToFire) {
									_p3 = true;
								}
							}
							_StartDoComplete = true;
							timer.Stop();
							timer.Reset();
						}
					}
				break;


				case 2: // 6 ball auto, 2 lots of three balls 
					if (!_StartDoComplete) {	
						timer.Start();
						if (timer.Get() <= Ball3Shoot){
							AutoAimToFire(dt);
							if (ReadyToFire) {
								_p1 = true;
							}
							_StartDoComplete = true;
							timer.Stop();
							timer.Reset();
						}	

						if (_strt) {
							timer.Start();
							if (timer.Get() <= Ball3Shoot){
							AutoAimToFire(dt);
								if (ReadyToFire) {
									_p2 = true;
								}
							}
							_StartDoComplete = true;
							timer.Stop();
							timer.Reset();
						}

					}
				break;


				case 3: // 3 ball left auto , this is at the end 
					if (_strt) {
						timer.Start();
						if (timer.Get() <= Ball3Shoot) {
							AutoAimToFire(dt);
						} else {
							FlyWheelPower = 0;
						}
						_FlyWheel.transmission->SetVoltage(12 * FlyWheelPower);
						timer.Stop();
						timer.Reset();
						_StartDoComplete = false;
					}
				break;

				
				case 4: // 3 ball mid auto
					if (_strt) {
						timer.Start();
						if (timer.Get() <= Ball3Shoot) {
							AutoAimToFire(dt);
						} else {
							FlyWheelPower = 0;
						}
						_FlyWheel.transmission->SetVoltage(12 * FlyWheelPower);
						timer.Stop();
						timer.Reset();
						//end of auto
					}
				break;


				case 5: // 3 ball right auto
					if (_strt) {
						timer.Start();
						if (timer.Get() <= Ball3Shoot) {
							AutoAimToFire(dt);
						} else {
							FlyWheelPower = 0;
						}
						_FlyWheel.transmission->SetVoltage(12 * FlyWheelPower);
						timer.Stop();
						timer.Reset();
						_StartDoComplete = false;
					}
				break;


				case 6: // make it go to case 2 automatically 
					TurretStop = true;
					// TurretAutoSelection++; // what??
				break;
			}
		break;
	}		
}


// @TODO Turret Test
void Turret::TestOnUpdate(double dt) {

	_RotationalAxis.transmission->SetVoltage(12 * RotationPower);
	_VerticalAxis.transmission->SetVoltage(12 * AngularPower);
	_FlyWheel.transmission->SetVoltage(12 * FlyWheelPower);

}