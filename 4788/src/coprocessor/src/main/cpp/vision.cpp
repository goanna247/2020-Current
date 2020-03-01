#include "vision.h"
#include <iostream>
// #include <windows.h>

double offsetX, offsetY;
int ResWidth = 640, ResHeight = 480;

cv::Point RotationLineStrt(320, 0);
cv::Point RotationLineEnd(320, 480);

double cx, cy;
double Lastcx = 0, Lastcy = 0;
double SecondLastcx = 0, SecondLastcy = 0;

cv::Mat Image; // Origin Image
cv::Mat TrackingImage; // Imaged after it has been filtered
cv::Mat ProcessingOutput; // Image after is has been processed

void curtin_frc_vision::run() {
	auto inst = nt::NetworkTableInstance::GetDefault();
	auto visionTable = inst.GetTable("VisionTracking");
	auto table = visionTable->GetSubTable("Target");

	TargetX = table->GetEntry("Target_X");
	TargetY = table->GetEntry("Target_Y");
	ImageHeight = table->GetEntry("ImageHeight");
	ImageWidth = table->GetEntry("ImageWidth");

	inst.StartClientTeam(4788);


	vision.SetupVision(&Image, 0, 60, ResHeight, ResWidth, 1, "Turret Cam", true);
	vision.CustomTrack(&TrackingImage, &Image, 50, 70, 250, 255, 30, 255, 1 ,1);
	vision.Processing.visionHullGeneration.BoundingBox(&TrackingImage, &ProcessingOutput, &cx, &cy, 10);
	#ifdef __DESKTOP__ 
	std::cout << "Exposure Might be dissabled on local machine" << std::endl;
	#else
	system("v4l2-ctl -d /dev/video0 --set-ctrl=exposure_absolute=1");
	#endif
	std::cout << "Vision Tracking Process Running" << std::endl;
	std::cout << "This thing is working" << std::endl;
	while (true) {
		if (vision.Camera.cam.sink.GrabFrame(Image) != 0) {
		
			offsetX = cx-(ResWidth/2);
			offsetY = cy; // Don't need offset. We're using setpoints



			double AverageCX = (offsetX + Lastcx)/2;

			// Display Image
			cv::line(ProcessingOutput, RotationLineStrt, RotationLineEnd, cv::Scalar(255,0,255), 2);
			vision.Display("Output", &ProcessingOutput);

			visionTable->PutBoolean("Vision Active", true);

			TargetX.SetDouble(AverageCX);
			TargetY.SetDouble(offsetY);
			ImageHeight.SetDouble(ResHeight);
			ImageWidth.SetDouble(ResWidth);

			Lastcx = offsetX;
			Lastcy = offsetY;
		} else {
			visionTable->PutBoolean("Vision Active", false);
		}
	}
}