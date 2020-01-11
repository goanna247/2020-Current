#include "vision.h"
#include <iostream>

double offsetX, offsetY;
int ResWidth = 640, ResHeight = 480;

double cx, cy;

cv::Mat Image; // Origin Image
cv::Mat TrackingImage; // Imaged After it has been procesed
cv::Mat ProcessingOutput;

void curtin_frc_vision::run() {

	auto inst = nt::NetworkTableInstance::GetDefault();
	auto visionTable = inst.GetTable("VisionTracking");
	auto table = visionTable->GetSubTable("Target");

	TargetX = table->GetEntry("Target_X");
	TargetY = table->GetEntry("Target_Y");

	vision.SetupVision(&Image, 0, 30, ResHeight, ResWidth, 100, "TestCam", true);
	vision.CustomTrack(&TrackingImage, &Image, 30, 70, 50, 255, 100, 0, 0);
	cv::waitKey(5000);
	vision.Processing.visionHullGeneration.BoundingBox(&TrackingImage, &ProcessingOutput, &cx, &cy, 10);
	while (true) {
		if (vision.Camera.cam.sink.GrabFrame(Image) != 0) {

			// Vision Outputing
			vision.Output.Display("Origin Image", &Image);
			vision.Output.Display("Green Filtered Image", &TrackingImage);
			vision.Output.Display("Contour Detection", &ProcessingOutput);

			//Calc offset
			offsetX = cx-(ResWidth/2);
			offsetY = cy-(ResHeight/2);


			// Sending Values to nt
			TargetX.SetDouble(offsetX);
			TargetY.SetDouble(offsetY);
		}
	}
}