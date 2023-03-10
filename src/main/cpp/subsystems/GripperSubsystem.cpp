#include "subsystems/GripperSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

GripperSubsystem::GripperSubsystem() :
    Arm("Gripper", GRIPPER)
{

}

void GripperSubsystem::Periodic() {
    Arm::Periodic();
    frc::SmartDashboard::PutNumber("Amperage", Arm::GetAmperage());
}