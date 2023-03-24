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
}

void GripperSubsystem::Periodic() {
    Arm::Periodic();
    // Arm::SetRaw(gripperPower);
    // frc::SmartDashboard::PutNumber("Arm Amperage", Arm::GetAmperage());
}

bool GripperSubsystem::Set(double power) {
    // if(power == 0) power = 0.026;
    // if(std::abs(power) < 0.1) power = 0;
    // if(Arm::Set(power)) return true;
    // Arm::SetSetpoint(Arm::GetPosition());
    // return false;
    // gripperPower = power;
    // wpi::outs() << power;
    // wpi::outs() << "\n";
    Arm::GetMotor()->Set(power);

    return false;
}