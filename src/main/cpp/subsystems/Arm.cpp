#include "subsystems/Arm.h"

#include <frc/shuffleboard/Shuffleboard.h>

Arm::Arm(std::string _name, int canID) :
    name{_name},
    motor{canID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    encoder{motor.GetAbsoluteEncoder(rev::SparkMaxAbsoluteEncoder::Type::kDutyCycle)}
{
    encoderPosition = frc::Shuffleboard::GetTab("Telemetry").Add(name + " Encoder Position", 0.0).GetEntry();
    setPower = frc::Shuffleboard::GetTab("Telemetry").Add(name + " Power", 0.0).GetEntry();
}

void Arm::Periodic() {
    encoderPosition->SetDouble(GetPosition());
    setPower->SetDouble(power);
}

void Arm::Set(double power) {
    Arm::power = power;
}

double Arm::GetPosition() {
    return encoder.GetPosition();
}

double Arm::GetAmperage() {
    return motor.GetOutputCurrent();
}