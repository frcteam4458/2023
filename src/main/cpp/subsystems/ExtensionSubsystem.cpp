#include "subsystems/ExtensionSubsystem.h"

ExtensionSubsystem::ExtensionSubsystem() :
    Arm("Extension ", EXTENSION)
{
}

void ExtensionSubsystem::Periodic() {
    Arm::Periodic();
}