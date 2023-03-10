#pragma once

#ifndef ROBOT_H
#define ROBOT_H

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "RobotContainer.h"

class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  void AutonomousInit() override;
  void AutonomousPeriodic() override;
  void TeleopInit() override;
  void TeleopPeriodic() override;
  // void SimulationInit() override;
  // void SimulationPeriodic() override;

 private:
  frc2::Command* autonomousCommand = nullptr;

  RobotContainer container;
};

#endif