#include "Turret.h"
#include <iostream>

using namespace wml;
using namespace wml::controllers;

// Bulk of the code that is used but never looked at... unless you look at it


// Distance Setpoints for Turret Angle
double Turret::SetPointSelection(double LowPoint, double MaxPoint, double PixleAmount, double TargetInput) {
  double targetEncoderValue;

  // Setpoint Selection.
	targetEncoderValue = (TargetInput < LowPoint) && (TargetInput > LowPoint + PixleAmount) ? ControlMap::AngleSetpoint1 : MaxPoint;
	LowPoint += PixleAmount;
	targetEncoderValue = (TargetInput < LowPoint) && (TargetInput > LowPoint + PixleAmount) ? ControlMap::AngleSetpoint2 : MaxPoint;
	LowPoint += PixleAmount;
	targetEncoderValue = (TargetInput < LowPoint) && (TargetInput > LowPoint + PixleAmount) ? ControlMap::AngleSetpoint3 : MaxPoint;
	LowPoint += PixleAmount;
	targetEncoderValue = (TargetInput < LowPoint) && (TargetInput > LowPoint + PixleAmount) ? ControlMap::AngleSetpoint4 : MaxPoint;
	LowPoint += PixleAmount;
	targetEncoderValue = (TargetInput < LowPoint) && (TargetInput > LowPoint + PixleAmount) ? ControlMap::AngleSetpoint5 : MaxPoint;
	LowPoint += PixleAmount;
	targetEncoderValue = (TargetInput < LowPoint) && (TargetInput > LowPoint + PixleAmount) ? ControlMap::AngleSetpoint6 : MaxPoint;
	LowPoint += PixleAmount;
	targetEncoderValue = (TargetInput < LowPoint) && (TargetInput > LowPoint + PixleAmount) ? ControlMap::AngleSetpoint7 : MaxPoint;
	LowPoint += PixleAmount;
	targetEncoderValue = (TargetInput < LowPoint) && (TargetInput > LowPoint + PixleAmount) ? ControlMap::AngleSetpoint8 : MaxPoint;
	LowPoint += PixleAmount;
	targetEncoderValue = (TargetInput < LowPoint) && (TargetInput > LowPoint + PixleAmount) ? ControlMap::AngleSetpoint9 : MaxPoint;
	LowPoint += PixleAmount;
	targetEncoderValue = (TargetInput < LowPoint) && (TargetInput > LowPoint + PixleAmount) ? ControlMap::AngleSetpoint10 : MaxPoint;
	LowPoint += PixleAmount;

  return targetEncoderValue;
}



// X Auto Aim Algorithm
double Turret::XAutoAimCalc(double dt, double targetx)  {

	double TurretFullRotation = (ControlMap::TurretEncoderRotations * ControlMap::TurretRatio);
	double Rotations2FOV = (TurretFullRotation/ControlMap::CamFOV);
	double targetXinRotations = targetX * (Rotations2FOV/imageWidth);

	Rgoal = _RotationalAxis.encoder->GetEncoderRotations() + targetXinRotations;
	double input = _RotationalAxis.encoder->GetEncoderRotations();

	// Calculate PID
	Rerror = Rgoal - input;

	double derror = (Rerror - RpreviousError) / dt;
	Rsum = Rsum + Rerror * dt;

	if (Rsum > (imageWidth/2)) {
		Rsum = imageWidth;
	} else if (Rsum < -(imageWidth/2)) {
		Rsum = -imageWidth;
	}

	double output = RkP * Rerror + RkI * Rsum + RkD * derror;

	// Convert to -1 - 1 for motor
	output /= Rotations2FOV;

	table->PutNumber("RoationDError", derror);
	table->PutNumber("RotationError", Rerror);
	table->PutNumber("RotationDelta Time", dt);
	table->PutNumber("RotationOutput", output);

	RpreviousError = Rerror;
	return -output;
}


// Tuning Functions

void Turret::TuneAnglePID() {

	if (_contGroup.Get(ControlMap::kpUP, Controller::ONRISE)) {
		AkP += 0.01;
	} else if (_contGroup.Get(ControlMap::kpDOWN, Controller::ONRISE)) {
		AkP -= 0.001;
	} else if (_contGroup.Get(ControlMap::kiUP, Controller::ONRISE)) {
		AkI += 0.001;
	} else if (_contGroup.Get(ControlMap::kiDOWN, Controller::ONRISE)) {
		AkI -= 0.001;
	} else if (_contGroup.Get(ControlMap::kdUP, Controller::ONRISE)) {
		AkD += 0.001;
	}	else if (_contGroup.Get(ControlMap::kdDOWN, Controller::ONRISE)) {
		AkD -= 0.001;
	}

	table->PutNumber("kP", AkP);
	table->PutNumber("kI", AkI);
	table->PutNumber("kD", AkD);
	table->PutNumber("Sum", Asum);
}

// Turret PID Tuning
void Turret::TuneTurretPID() {

	if (_contGroup.Get(ControlMap::kpUP, Controller::ONRISE)) {
		RkP += 0.01;
	} else if (_contGroup.Get(ControlMap::kpDOWN, Controller::ONRISE)) {
		RkP -= 0.001;
	} else if (_contGroup.Get(ControlMap::kiUP, Controller::ONRISE)) {
		RkI += 0.001;
	} else if (_contGroup.Get(ControlMap::kiDOWN, Controller::ONRISE)) {
		RkI -= 0.001;
	} else if (_contGroup.Get(ControlMap::kdUP, Controller::ONRISE)) {
		RkD += 0.001;
	}	else if (_contGroup.Get(ControlMap::kdDOWN, Controller::ONRISE)) {
		RkD -= 0.001;
	}

	table->PutNumber("kP", RkP);
	table->PutNumber("kI", RkI);
	table->PutNumber("kD", RkD);
	table->PutNumber("Sum", Rsum);
}

// Zeroing To Left
void Turret::TurretZeroLeft(double Time) {
  while (_LeftLimit.Get() < 1) {
		if (ZeroTimer.Get() < ControlMap::TurretZeroTimeoutSeconds) {
			_RotationalAxis.transmission->SetVoltage(12 * -0.3);
		} else {
			std::cout << "Turret Zero Timed Out" << std::endl;
			_contGroup.GetController(ControlMap::CoDriver).SetRumble(wml::controllers::RumbleType::kLeftRumble, 1);
			_contGroup.GetController(ControlMap::CoDriver).SetRumble(wml::controllers::RumbleType::kRightRumble, 1);
			_RotationalAxis.transmission->SetVoltage(0);
			break;
		}
	} 
  _RotationalAxis.encoder->ZeroEncoder();
}

// Zeroing To Right
void Turret::TurretZeroRight(double Time) {
  while (_RightLimit.Get() < 1) {
		if (ZeroTimer.Get() < ControlMap::TurretZeroTimeoutSeconds) {
			_RotationalAxis.transmission->SetVoltage(12 * 0.3);
		} else {
			std::cout << "Turret Zero Timed Out" << std::endl;
			_contGroup.GetController(ControlMap::CoDriver).SetRumble(wml::controllers::RumbleType::kLeftRumble, 1);
			_contGroup.GetController(ControlMap::CoDriver).SetRumble(wml::controllers::RumbleType::kRightRumble, 1);
			_RotationalAxis.transmission->SetVoltage(0);
			break;
		}
	}
}

// Zero Angle
void Turret::TurretZeroAngle(double Time) {
  while(_AngleDownLimit.Get() < 1) {
		if (ZeroTimer.Get() < ControlMap::TurretZeroTimeoutSeconds) {
			_VerticalAxis.transmission->SetVoltage(12 * 0.2);
		} else {
			_VerticalAxis.transmission->SetVoltage(0);
			_contGroup.GetController(ControlMap::CoDriver).SetRumble(wml::controllers::RumbleType::kLeftRumble, 1);
			_contGroup.GetController(ControlMap::CoDriver).SetRumble(wml::controllers::RumbleType::kRightRumble, 1);
			std::cout << "Turret Zero Timed Out" << std::endl;
			break;
		}
	}
  _VerticalAxis.encoder->ZeroEncoder();
}


// Feedback for correct flywheel speeds
void Turret::ContFlywheelFeedback() {
  if (_FlyWheel.encoder->GetEncoderAngularVelocity() >= ControlMap::FlyWheelVelocity) {
		_contGroup.GetController(ControlMap::CoDriver).SetRumble(wml::controllers::RumbleType::kLeftRumble, 1);
		_contGroup.GetController(ControlMap::CoDriver).SetRumble(wml::controllers::RumbleType::kRightRumble, 1);
	} else {
		_contGroup.GetController(ControlMap::CoDriver).SetRumble(wml::controllers::RumbleType::kLeftRumble, 0);
		_contGroup.GetController(ControlMap::CoDriver).SetRumble(wml::controllers::RumbleType::kRightRumble, 0);
	}
}

// Tuner Control
void Turret::PIDTuner() {
  if (ControlMap::TuneTurretPID && ControlMap::TuneAnglePID) {
		std::cout << "Conflict Turret/Angle Tuning" << std::endl;
	}	else if (ControlMap::TuneTurretPID) {
		TuneTurretPID();
	} else if (ControlMap::TuneAnglePID) {
		TuneAnglePID();
	}
}

// FlyWheel Code
void Turret::FlyWheelManualSpinup() {
	FlyWheelPower = _contGroup.Get(ControlMap::TurretFlyWheelSpinUp);
}

void Turret::FlyWheelAutoSpinup() {
	FlyWheelPower += _FlyWheel.encoder->GetEncoderAngularVelocity() < ControlMap::FlyWheelVelocity ? 0.01 : 0;
}