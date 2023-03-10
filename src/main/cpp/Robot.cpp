#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

// Gets autonomous command from RobotContainer, schedules if command is not null
void Robot::AutonomousInit() {
  autonomousCommand = container.GetAutonomousCommand();

  if (autonomousCommand != nullptr) {
    autonomousCommand->Schedule();
  }
}

void Robot::AutonomousPeriodic() {}

// Destroys autonomous command, gets teleop from RobotContainer, schedules if command is not null
void Robot::TeleopInit() {
  if (autonomousCommand != nullptr) {
    autonomousCommand->Cancel();
    autonomousCommand = nullptr;
  }

  container.GetTeleopCommand()->Schedule();
}

void Robot::TeleopPeriodic() {}

#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
