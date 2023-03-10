#include "commands/GripCommand.h"

#include <frc/shuffleboard/Shuffleboard.h>

#include <frc/smartdashboard/SmartDashboard.h>

static bool closed = false;
static double amperage = 0.0;

GripCommand::GripCommand(GripperSubsystem* subsystem, double _amperage)
    : gripperSubsystem{subsystem}
{

    AddRequirements(gripperSubsystem);
    amperage = _amperage;
    
}

void GripCommand::Initialize() {
    closed = !closed;
    frc::SmartDashboard::PutBoolean("Closed?", closed);
}

void GripCommand::Execute() {
    if(closed) gripperSubsystem->Set(0.1);
}

void GripCommand::End(bool interrupted) {
    gripperSubsystem->Set(0);
}

bool GripCommand::GetClosed() {
    return closed;
}