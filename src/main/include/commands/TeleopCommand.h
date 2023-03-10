#pragma once

#ifndef TELEOP_COMMAND_H
#define TELEOP_COMMAND_H

#include <frc2/command/CommandBase.h>
#include <frc2/command/CommandHelper.h>

#include "subsystems/DriveSubsystem.h"

#include <frc/XboxController.h>
#include <frc/Joystick.h>
#include <frc/smartdashboard/SendableChooser.h>

class TeleopCommand
    : public frc2::CommandHelper<frc2::CommandBase, TeleopCommand> {
 public:
  explicit TeleopCommand(DriveSubsystem* subsystem);
  void Execute() override;

 private:
  DriveSubsystem* driveSubsystem;
  frc::XboxController controller{0};
  frc::SendableChooser<bool> driveChooser;
};

#endif