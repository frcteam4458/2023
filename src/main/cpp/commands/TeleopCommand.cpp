#include "commands/TeleopCommand.h"

#include <frc/shuffleboard/Shuffleboard.h>

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
        driveSubsystem->Drive(-controller.GetLeftY(), controller.GetRawAxis(2));
}