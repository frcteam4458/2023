#include "subsystems/GripperSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>


GripperSubsystem::GripperSubsystem() :
    Arm("Gripper", GRIPPER)
{
    Arm::GetMotor()->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    Arm::GetPID().SetP(0.05);
    Arm::SetSetpoint(Arm::GetPosition());
    Arm::GetMotor()->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 5.0);
    Arm::GetMotor()->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
}

void GripperSubsystem::Periodic() {
    Arm::Periodic();
    frc::SmartDashboard::PutNumber("Arm Amperage", Arm::GetAmperage());
}

bool GripperSubsystem::Set(double power) {
    if(std::abs(power) < 0.1) power = 0;
    if(Arm::Set(power)) return true;
    Arm::SetSetpoint(Arm::GetPosition());
    return false;
}