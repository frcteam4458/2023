#pragma once

#ifndef EXTENSION_SUBSYSTEM_H
#define EXTENSION_SUBSYSTEM_H

#include "subsystems/Arm.h"
#include "Constants.h"
#include <rev/CANSparkMax.h>

#include <networktables/GenericEntry.h>

class ExtensionSubsystem : public Arm {
 public:
  ExtensionSubsystem();
  void Periodic() override;

  bool GetSoftLimits();
  void SetSoftLimits(bool enabled);
  bool Set(double power) override;
 private:
    nt::GenericEntry* softLimits;
};

#endif