#pragma once

#include "Dashboard/Orientation.h"

class Drive {
  public:
    void DriveImage(cv::Mat *Window, nt::NetworkTableInstance *inst, int posX, int posY);
  private:
    // Network table
    nt::NetworkTableEntry LPower;
    nt::NetworkTableEntry RPower;
    nt::NetworkTableEntry LEC;
    nt::NetworkTableEntry REC;
};