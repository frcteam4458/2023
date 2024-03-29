#pragma once

#ifndef ROBOT_H
#define ROBOT_H

#include <optional>

#include <frc/TimedRobot.h>
#include <frc2/command/Command.h>
#include <frc2/command/CommandPtr.h>

#include "RobotContainer.h"

#include <frc/DriverStation.h>
#include <frc/AddressableLED.h>

#include <rev/CANSparkMax.h>



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
  std::optional<frc2::CommandPtr> autonomousCommand;
  RobotContainer container;
  int c = 0;
  std::array<frc::AddressableLED::LEDData, 60> buffer;
  frc::DriverStation::Alliance alliance{frc::DriverStation::Alliance::kInvalid};
  // rev::CANSparkMax gripper{7, rev::CANSparkMax::MotorType::kBrushless};
};

#endif