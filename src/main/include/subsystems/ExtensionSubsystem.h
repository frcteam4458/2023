#pragma once

#ifndef EXTENSION_SUBSYSTEM_H
#define EXTENSION_SUBSYSTEM_H

#include "subsystems/Arm.h"
#include "Constants.h"
#include <rev/CANSparkMax.h>

class ExtensionSubsystem : public Arm {
 public:
  ExtensionSubsystem();
  void Periodic() override;
 private:
};

#endif