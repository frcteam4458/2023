#include "subsystems/PivotSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <wpi/raw_ostream.h>


PivotSubsystem::PivotSubsystem() :
    Arm("Pivot", LIFT),
    encoder{PIVOT_ENCODER}
{
    Arm::SetInverted(true);

    Arm::GetMotor()->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
    Arm::GetMotor()->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, true);
    Arm::GetMotor()->SetSoftLimit(rev::CANSparkMax::SoftLimitDirection::kReverse, -51);
    SetSetpoint(Arm::GetPosition());
    Arm::GetPID().SetOutputRange(-0.4, 0.4);

    Arm::GetPID().SetP(0.1);

    Arm::SetPID(0.1, 0, 0);
    Arm::SetPIDRange(-0.4, 0.4);
}

void PivotSubsystem::Periodic() {

    Arm::Periodic();
    // if(PIVOT_MAX < encoder.GetDistance()) Set(0);
    // frc::SmartDashboard::PutNumber("Pivot Amps", Arm::GetAmperage());
    // frc::SmartDashboard::PutNumber("Pivot Spark Encoder", Arm::GetPosition());
    // frc::SmartDashboard::PutNumber("Pivot Abs Encoder", encoder.GetDistance());
}

bool PivotSubsystem::Set(double power) {
    if(std::abs(power) < 0.5) power = std::clamp(power, -0.5, 0.5);
    if(Arm::Set(power)) return true;
        SetSetpoint(Arm::GetPosition());
    return false;
}