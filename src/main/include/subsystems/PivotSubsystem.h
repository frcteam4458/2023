#pragma once

#ifndef LIFT_SUBSYSTEM_H
#define LIFT_SUBSYSTEM_H

#include "subsystems/Arm.h"
#include "Constants.h"
#include <rev/CANSparkMax.h>

class PivotSubsystem : public Arm {
 public:
  PivotSubsystem();
  void Periodic() override;
 private:
};

#endif