#pragma once

#ifndef ARM_h
#define ARM_H

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"
#include <rev/CANSparkMax.h>
#include <rev/sim/CANSparkMax.h>

#include <networktables/GenericEntry.h>

#include <string>

#include <rev/SparkMaxPIDController.h>

class Arm : public frc2::SubsystemBase {
 public:
  Arm(std::string _name, int canID);
  void Periodic() override;
  virtual bool Set(double power);
  double GetPosition();
  double GetAmperage();
  double GetCurrent250ms();
  double GetPower();
  void SetInverted(bool inverted);
  void SetSetpoint(double _setpoint);
  double GetSetpoint();
  rev::CANSparkMax* GetMotor();
  rev::SparkMaxPIDController GetPID();
  bool AtSetpoint();
  rev::SparkMaxRelativeEncoder GetEncoder();
 private:
  std::string name;
  nt::GenericEntry* encoderPosition;
  nt::GenericEntry* setPower;
  double power = 0;
  rev::CANSparkMax motor;
  rev::SparkMaxRelativeEncoder encoder;
  rev::SparkMaxPIDController pidController;
  double amperage250ms = 0;
  double position = -100; // -100 is default value
  double setpoint = -100; // -100 is default value
};

#endif