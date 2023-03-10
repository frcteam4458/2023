#pragma once

#ifndef ROBOT_CONTAINER_H
#define ROBOT_CONTAINER_H

#include <frc2/command/Command.h>

#include "commands/TeleopCommand.h"
#include "commands/GripCommand.h"

#include "subsystems/DriveSubsystem.h"
#include "subsystems/ExtensionSubsystem.h"
#include "subsystems/GripperSubsystem.h"
#include "subsystems/PivotSubsystem.h"

#include <frc2/command/RamseteCommand.h>
#include <frc/controller/RamseteController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/SimpleMotorFeedforward.h>
#include <frc/trajectory/Trajectory.h>
#include <frc/trajectory/TrajectoryConfig.h>
#include <frc/trajectory/TrajectoryGenerator.h>


class RobotContainer {
 public:  
  RobotContainer();

  frc2::Command* GetTeleopCommand();
  frc2::Command* GetAutonomousCommand();

 private:
  DriveSubsystem driveSubsystem;
  ExtensionSubsystem extensionSubsystem;
  GripperSubsystem gripperSubsystem;
  PivotSubsystem pivotSubsystem;

  TeleopCommand teleopCommand;

  void ConfigureButtonBindings();
};

#endif