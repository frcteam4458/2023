#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include <frc2/command/Command.h>

#include <frc/smartdashboard/SmartDashboard.h>



#include <array>


void Robot::RobotInit() {
  container.GetLEDStrip()->SetLength(60);
  for(int i = 0; i < 60; i++) {
    buffer[i].SetRGB(127, 0, 127);
  }

  container.GetLEDStrip()->SetData(buffer);

  container.GetLEDStrip()->Start();
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();
}

// Gets autonomous command from RobotContainer, schedules if command is not null
void Robot::AutonomousInit()
{
  frc::SmartDashboard::PutBoolean("disabled", false);
  autonomousCommand = container.GetAutonomousCommand();

  if(autonomousCommand)
  autonomousCommand->Schedule();
}

void Robot::DisabledPeriodic() {
  if(frc::DriverStation::GetAlliance() != alliance) {
    container.GetLEDStrip()->SetLength(60);
  for(int i = 0; i < 60; i++) {
    buffer[i].SetRGB((frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed) * 127, 0,
    (frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kBlue) * 127);
  }

  container.GetLEDStrip()->SetData(buffer);

  container.GetLEDStrip()->Start();
  }
  frc::SmartDashboard::PutBoolean("disabled", true);
}

// void Robot::AutonomousPeriodic() {

// }

// // Destroys autonomous command, gets teleop from RobotContainer, schedules if command is not null
void Robot::TeleopInit() {
  frc::SmartDashboard::PutBoolean("disabled", false);
  if(autonomousCommand)
    autonomousCommand->Cancel();
  container.GetTeleopCommand()->Schedule();
  // gripper.Set(-0.5);
}

// void Robot::TeleopPeriodic() {}



#ifndef RUNNING_FRC_TESTS
int main() {
  return frc::StartRobot<Robot>();
}
#endif
