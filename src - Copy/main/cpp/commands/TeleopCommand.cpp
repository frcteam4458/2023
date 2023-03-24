#include "commands/TeleopCommand.h"

#include <frc/shuffleboard/Shuffleboard.h>

#include <math.h>

TeleopCommand::TeleopCommand(DriveSubsystem* subsystem)
    : driveSubsystem{subsystem},
    driveChooser{}
{
    driveChooser.SetDefaultOption("Curvature", true);
    driveChooser.AddOption("Differential", false);
    frc::Shuffleboard::GetTab("Dashboard").Add(driveChooser).WithPosition(0, 0).WithSize(2, 1);
}

void TeleopCommand::Execute() {
    // if(driveChooser.GetSelected())
        // driveSubsystem->DriveCurvature(-controller.GetLeftY(), controller.GetRawAxis(2)); // axis 2 is right X
    // else

    y = controller.GetLeftY();
    omega = controller.GetRawAxis(2);
    if(controller.GetRawButton(8) != 0) omega /= 4;
        driveSubsystem->Drive(-std::pow(y, 3), omega/8);
}