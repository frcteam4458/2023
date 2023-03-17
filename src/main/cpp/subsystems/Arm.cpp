#include "subsystems/Arm.h"

#include <frc/shuffleboard/Shuffleboard.h>

#include <thread>
#include <chrono>
#include <math.h>

#include <frc/smartdashboard/SmartDashboard.h>

Arm::Arm(std::string _name, int canID) :
    name{_name},
    motor{canID, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    encoder{motor.GetEncoder()},
    pidController{motor.GetPIDController()}
{
    encoderPosition = frc::Shuffleboard::GetTab("Telemetry").Add(name + " Encoder Position", 0.0).GetEntry();
    setPower = frc::Shuffleboard::GetTab("Telemetry").Add(name + " Power", 0.0).GetEntry();

    std::thread smoothCurrentThread{
        [this] {
            double amps[] = {0, 0, 0, 0};
            double total = 0;
            while(true) {
                for(int i = 0; i < 4; i++) {
                    amps[i] = motor.GetOutputCurrent();
                    std::this_thread::sleep_for(std::chrono::milliseconds(50));
                }
                for(int i = 0; i < 4; i++) {
                    total += amps[i];
                    total /= 4;
                }
                amperage250ms = total;
                total = 0;
            }
        }
    };
    smoothCurrentThread.detach(); // i wanna kill myself
        position = encoder.GetPosition();
}

void Arm::Periodic() {
    encoderPosition->SetDouble(GetPosition());
    
    if(frc::SmartDashboard::GetBoolean("disabled", true)) {
        setpoint = GetPosition();
    }
}

bool Arm::Set(double power) {
    if(std::abs(power) < 0.025) {
        power = 0;
        Arm::GetPID().SetReference(GetSetpoint(), rev::CANSparkMax::ControlType::kPosition);
        return true;
    }
    position = encoder.GetPosition();
    Arm::power = power;
    motor.Set(power);
    setPower->SetDouble(power);
    return false;
}

double Arm::GetPosition() {
    return encoder.GetPosition();
}

double Arm::GetAmperage() {
    return motor.GetOutputCurrent();
}

double Arm::GetCurrent250ms() {
    return amperage250ms;
}

// NOT WATTS! [-1,1]V
double Arm::GetPower() {
    return motor.Get();
}

void Arm::SetInverted(bool inverted) {
    motor.SetInverted(inverted);
}

rev::CANSparkMax* Arm::GetMotor() {
    return &motor;
}

rev::SparkMaxPIDController Arm::GetPID() {
    return pidController;
}

void Arm::SetSetpoint(double _setpoint) {
    setpoint = _setpoint;
}

double Arm::GetSetpoint() {
    return setpoint;
}

bool Arm::AtSetpoint() { // idk what im doing
    if(encoder.GetVelocity() < 0.25) {
        if(setpoint - 0.5 < encoder.GetPosition() && encoder.GetPosition() < setpoint + 0.5) return true;
    }

    return false;
}

rev::SparkMaxRelativeEncoder Arm::GetEncoder() {
    return encoder;
}