#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include <frc2/command/Command.h>

#include <frc/smartdashboard/SmartDashboard.h>

void Robot::RobotInit() {}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

// Gets autonomous command from RobotContainer, schedules if command is not null
void Robot::AutonomousInit()
{
  frc::SmartDashboard::PutBoolean("disabled", false);
  autonomousCommand = container.GetAutonomousCommand();
  autonomousCommand->Schedule();
}

void Robot::DisabledPeriodic() {
  frc::SmartDashboard::PutBoolean("disabled", true);
}

// void Robot::AutonomousPeriodic() {

// }

// // Destroys autonomous command, gets teleop from RobotContainer, schedules if command is not null
void Robot::TeleopInit() {
  frc::SmartDashboard::PutBoolean("disabled", false);
  container.GetTeleopCommand()->Schedule();
}

// void Robot::TeleopPeriodic() {}



#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
