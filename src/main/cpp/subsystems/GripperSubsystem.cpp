#include "subsystems/GripperSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>


GripperSubsystem::GripperSubsystem() :
    Arm("Gripper", GRIPPER)
{
    Arm::GetMotor()->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    Arm::GetPID().SetP(0.05);
    Arm::SetSetpoint(Arm::GetPosition());
}

void GripperSubsystem::Periodic() {
    Arm::Periodic();
    frc::SmartDashboard::PutNumber("Arm Amperage", Arm::GetAmperage());
}

bool GripperSubsystem::Set(double power) {
    if(Arm::Set(power)) return true;
    Arm::SetSetpoint(Arm::GetPosition());
    return false;
}