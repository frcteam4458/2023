#include "commands/GripCommand.h"

#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/smartdashboard/SmartDashboard.h>

static bool closed = false;
static double amperage = 0.0;

GripCommand::GripCommand(GripperSubsystem* subsystem, double _amperage)
    : gripperSubsystem{subsystem},
    gripperController{gripperkP, 0.0, 0.0, 200_ms}
{

    AddRequirements(gripperSubsystem);
    amperage = _amperage;
    
}

void GripCommand::Initialize() {
    closed = !closed;
    frc::SmartDashboard::PutBoolean("Closed?", closed);
}

void GripCommand::Execute() {
    if(!closed) {
        if(gripperSubsystem->GetPosition() < 0) {
            gripperSubsystem->Set(-1);
        } else {
            gripperSubsystem->Set(0);
        }
        return;
    }
    gripperSubsystem->Set(0.2);
    // gripperSubsystem->Set(gripperController.Calculate(gripperSubsystem->GetCurrent250ms()))
}

void GripCommand::End(bool interrupted) {
    gripperSubsystem->Set(0);
}

bool GripCommand::GetClosed() {
    return closed;
}