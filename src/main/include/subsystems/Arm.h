#pragma once

#ifndef ARM_h
#define ARM_H

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <rev/CANSparkMax.h>
#include <rev/sim/CANSparkMax.h>

#include <networktables/GenericEntry.h>

#include <string>

class Arm : public frc2::SubsystemBase {
 public:
  Arm(std::string _name, int canID);
  void Periodic() override;
  void Set(double power);
  double GetPosition();
  double GetAmperage();
 private:
  std::string name;
  nt::GenericEntry* encoderPosition;
  nt::GenericEntry* setPower;
  double power = 0;
  rev::CANSparkMax motor;
  rev::SparkMaxAbsoluteEncoder encoder;
};

#endif