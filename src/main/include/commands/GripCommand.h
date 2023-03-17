#pragma once

#ifndef GRIP_COMMAND_H
#define GRIP_COMMAND_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>
#include <frc2/command/Command.h>

#include "subsystems/GripperSubsystem.h"
#include <frc/controller/PIDController.h>

class GripCommand
    : public frc2::CommandHelper<frc2::CommandBase, GripCommand> {
 public:
  explicit GripCommand(GripperSubsystem* subsystem, double _amperage);
  
  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  static bool GetClosed();
 private:
  GripperSubsystem* gripperSubsystem;
  frc::PIDController gripperController;

  
};

#endif