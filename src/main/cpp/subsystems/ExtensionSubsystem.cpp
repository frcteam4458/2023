#include "subsystems/ExtensionSubsystem.h"

#include <frc/shuffleboard/Shuffleboard.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include <math.h>

ExtensionSubsystem::ExtensionSubsystem() :
    Arm("Extension ", EXTENSION)
{
    Arm::GetMotor()->RestoreFactoryDefaults();
    Arm::GetMotor()->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    Arm::GetMotor()->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    Arm::GetMotor()->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, 1);
    Arm::GetMotor()->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -43);
    softLimits = frc::Shuffleboard::GetTab("Dashboard").Add("Spooler Soft Limits", true).GetEntry();
    frc::SmartDashboard::PutNumber("Extension Spark Encoder", Arm::GetPosition());
    SetSetpoint(Arm::GetPosition());
    Arm::GetPID().SetP(0.1);
}

void ExtensionSubsystem::Periodic() {
    Arm::Periodic();
}

void ExtensionSubsystem::SetSoftLimits(bool enabled) {
    Arm::GetMotor()->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, enabled);
    Arm::GetMotor()->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, enabled);
    softLimits->SetBoolean(enabled);
}

bool ExtensionSubsystem::GetSoftLimits() {
    return Arm::GetMotor()->GetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward);
}

bool ExtensionSubsystem::Set(double power) {
    if(std::abs(power) < 0.16) power = std::clamp(power, -0.16, 0.16);
    if(Arm::Set(power)) return true;
    SetSetpoint(Arm::GetPosition());
    return false;
}