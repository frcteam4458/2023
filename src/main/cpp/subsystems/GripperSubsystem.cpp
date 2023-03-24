#include "subsystems/GripperSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <math.h>

#include <wpi/raw_ostream.h>


GripperSubsystem::GripperSubsystem() :
    Arm("Gripper", GRIPPER)
{
    Arm::GetMotor()->RestoreFactoryDefaults();
    // Arm::GetMotor()->SetIdleMode(rev::CANSparkMax::IdleMode::kBrake);
    Arm::GetPID().SetP(0.05);
    Arm::SetSetpoint(Arm::GetPosition());
    Arm::GetMotor()->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, -4);
    Arm::GetMotor()->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -16);
    Arm::GetMotor()->SetSmartCurrentLimit(4);
    Arm::GetMotor()->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);

    Arm::GetMotor()->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, false);

    Arm::GetWPIPID()->SetP(0.2);
    
}

void GripperSubsystem::Periodic() {
    Arm::Periodic();
}

bool GripperSubsystem::Set(double power) {
    Arm::GetMotor()->Set(power);
    return false;
}