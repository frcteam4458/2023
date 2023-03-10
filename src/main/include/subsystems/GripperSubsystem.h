#pragma once

#ifndef GRIPPER_SUBSYSTEM_H
#define GRIPPER_SUBSYSTEM_H

#include "subsystems/Arm.h"
#include "Constants.h"
#include <rev/CANSparkMax.h>

class GripperSubsystem : public Arm {
 public:
  GripperSubsystem();
  void Periodic() override;
 private:
};

#endif