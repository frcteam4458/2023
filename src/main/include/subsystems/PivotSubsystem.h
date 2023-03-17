#pragma once

#ifndef LIFT_SUBSYSTEM_H
#define LIFT_SUBSYSTEM_H

#include "subsystems/Arm.h"
#include "Constants.h"
#include <rev/CANSparkMax.h>
#include <frc/DutyCycleEncoder.h>
#include <rev/SparkMaxPIDController.h>

class PivotSubsystem : public Arm {
 public:
  PivotSubsystem();
  void Periodic() override;
  bool Set(double power) override;
      inline static bool isDisabled = true;

 private:
    frc::DutyCycleEncoder encoder;
};

#endif