#include "Robot.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/CommandScheduler.h>

#include <frc2/command/Command.h>

#include <frc/smartdashboard/SmartDashboard.h>


#include <array>


void Robot::RobotInit() {
  
}

void Robot::RobotPeriodic() {
  frc2::CommandScheduler::GetInstance().Run();

  c++;
  if(c % 50 == 0) {
   led.SetLength(60);
    std::array<frc::AddressableLED::LEDData, 60> buffer;
    bool red = frc::DriverStation::GetAlliance() == frc::DriverStation::Alliance::kRed;
    for(int i = 0; i < 60; i++) {
      // if(red)
      buffer[i].SetRGB(255, 0, 255);
      // else
      // buffer[i].SetRGB(0, 0, 255);
    }

    led.SetData(buffer);
  }
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


  // if(c % 50 == 0) {
   
  // }
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
