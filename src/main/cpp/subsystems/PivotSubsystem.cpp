#include "subsystems/PivotSubsystem.h"

PivotSubsystem::PivotSubsystem() :
    Arm("Lift", LIFT)
{

}

void PivotSubsystem::Periodic() {
    Arm::Periodic();
}