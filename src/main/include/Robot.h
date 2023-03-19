#pragma once

#ifndef ROBOT_H
#define ROBOT_H

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/AddressableLED.h>


class Robot : public frc::TimedRobot {
 public:
  void RobotInit() override;
  void RobotPeriodic() override;
  // void DisabledInit() override;
  void DisabledPeriodic() override;
  void AutonomousInit() override;
  // void AutonomousPeriodic() override;
  void TeleopInit() override;
  // void TeleopPeriodic() override;
  // void TestPeriodic() override;
  // void SimulationInit() override;
  // void SimulationPeriodic() override;
 private:
  frc2::Command* autonomousCommand = nullptr;
  frc::DriverStation::Alliance alliance{frc::DriverStation::Alliance::kInvalid};
  RobotContainer container;
  int c = 0;
  frc::AddressableLED led{0};

};

#endif