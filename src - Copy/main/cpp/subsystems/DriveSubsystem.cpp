#include "subsystems/DriveSubsystem.h"

#include <frc/smartdashboard/SmartDashboard.h>
#include <frc/shuffleboard/Shuffleboard.h>

#include <numbers>

#include <frc/motorcontrol/MotorControllerGroup.h>
#include <frc/drive/DifferentialDrive.h>

#include <units/angular_acceleration.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>
#include <units/voltage.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>

DriveSubsystem::DriveSubsystem() :

    #ifdef PWM_DRIVETRAIN
    fl{FL},
    fr{FR},
    bl{BL},
    br{BR},
    #endif

    #ifdef CAN_DRIVETRAIN
    fl{FL, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    fr{FR, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    bl{BL, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    br{BR, rev::CANSparkMaxLowLevel::MotorType::kBrushless},
    flEncoder{fl.GetEncoder()},
    frEncoder{fr.GetEncoder()},
    blEncoder{bl.GetEncoder()},
    brEncoder{br.GetEncoder()},
    #endif

    leftMotors{fl, bl},
    rightMotors{fr, br},

    s_fl{FL},
    s_fr{FR},
    s_bl{BL},
    s_br{BR},

    left{LEFT_ENCODER[0], LEFT_ENCODER[1]},
    right{RIGHT_ENCODER[0], RIGHT_ENCODER[1]},

    s_left{left},
    s_right{right},

    gyro{GYRO},
    s_gyro{gyro},

    // #ifdef __FRC_ROBORIO__
    pigeon{0},
    // #endif

    kinematics{trackWidth},
    // odometry{frc::Rotation2d{GetAngleDeg()}, 0_m, 0frc::Pose2d{}},
    odometry{frc::Rotation2d{GetAngleDeg()}, 0_m, 0_m, frc::Pose2d{frc::Translation2d{0_m, 0_m}, frc::Rotation2d{GetAngleDeg()}}},

    drive{leftMotors, rightMotors},

    drivetrainSim{ // this is pretty much direct from documentation
        // frc::DCMotor::CIM(2),
        // 7.29,
        // 6.5_kg_sq_m,
        // 60_kg,
        // 3_in,
        // 1_m,
        // {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
        // {0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005}

        frc::LinearSystemId::IdentifyDrivetrainSystem(
            units::volt_t{kV.value()} / 1_mps,
            units::volt_t{kA.value()} / 1_mps_sq,
            units::volt_t{akV.value()} / 1_rad_per_s,
            units::volt_t{akA.value()} / 1_rad_per_s_sq,
            trackWidth
        ),
        trackWidth,
        frc::DCMotor::CIM(2),
        10.71,
        3_in,
        {0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0}
    },
    
    field{}
{
    fr.SetInverted(true);
    br.SetInverted(true);
    right.SetReverseDirection(true);
    left.SetDistancePerPulse(2 * std::numbers::pi * 0.0762 / 360);
    right.SetDistancePerPulse(2 * std::numbers::pi * 0.0762 / 360);

    frc::SmartDashboard::PutData(&field);

        drive.SetSafetyEnabled(false);


    fl.SetOpenLoopRampRate(1);
    fr.SetOpenLoopRampRate(1);
    bl.SetOpenLoopRampRate(1);
    br.SetOpenLoopRampRate(1);

    fl.SetClosedLoopRampRate(1);
    fr.SetClosedLoopRampRate(1);
    bl.SetClosedLoopRampRate(1);
    br.SetClosedLoopRampRate(1);
    
}

void DriveSubsystem::Periodic() {
    odometry.Update(frc::Rotation2d{GetAngleDeg()}, units::meter_t{left.GetDistance()}, units::meter_t{right.GetDistance()});

    field.SetRobotPose(odometry.GetPose());

    // frc::SmartDashboard::PutNumber("FL", fl.Get());
    // frc::SmartDashboard::PutNumber("FR", fr.Get());
    // frc::SmartDashboard::PutNumber("BL", bl.Get());
    // frc::SmartDashboard::PutNumber("BR", br.Get());

    // frc::SmartDashboard::PutNumber("Odometer X", GetPose().X().value());
    // frc::SmartDashboard::PutNumber("Odometer X", GetPose().Y().value());

}

void DriveSubsystem::SimulationPeriodic() {
    drivetrainSim.SetInputs(fl.Get() * 12.0_V, fr.Get() * 12.0_V);
    drivetrainSim.Update(20_ms);
    s_left.SetDistance(drivetrainSim.GetLeftPosition().value());
    s_left.SetRate(drivetrainSim.GetLeftVelocity().value());
    s_right.SetDistance(drivetrainSim.GetRightPosition().value());
    s_right.SetRate(drivetrainSim.GetRightVelocity().value());

    s_gyro.SetAngle(drivetrainSim.GetHeading().Degrees().value());
    //s_gyro.SetRate((-drivetrainSim.GetHeading().Degrees().value() - -drivetrainSim.GetHeading().Degrees().value()) / 0.02);
}

void DriveSubsystem::Drive(float fwd, float omega) {
    leftMotors.Set(fwd + 2 * omega);
    rightMotors.Set(fwd - 2 * omega);
}

void DriveSubsystem::DriveVolts(units::volt_t left, units::volt_t right) {
    leftMotors.SetVoltage(left);
    rightMotors.SetVoltage(right);
}

void DriveSubsystem::DriveSpeeds(units::meters_per_second_t left, units::meters_per_second_t right) {
    auto leftV = leftFF.Calculate(left);
    auto rightV = rightFF.Calculate(right);
    leftMotors.SetVoltage(leftV);
    rightMotors.SetVoltage(rightV);
}

void DriveSubsystem::DriveCurvature(float fwd, float omega) {
    // frc::SmartDashboard::PutNumber("omega", omega);
    drive.CurvatureDrive(fwd, omega, true);
}

float DriveSubsystem::GetAngle() {
    #ifdef __FRC_ROBORIO__
    frc::SmartDashboard::PutNumber("pigeon", pigeon.GetYaw());
    return pigeon.GetYaw();
    #endif
    return gyro.GetAngle();
}

frc::DifferentialDriveWheelSpeeds DriveSubsystem::GetWheelSpeeds() {
    frc::DifferentialDriveWheelSpeeds speeds{};
    speeds.left = units::meters_per_second_t{left.GetRate()};
    speeds.right = units::meters_per_second_t{right.GetRate()};
    return speeds;
}

units::degree_t DriveSubsystem::GetAngleDeg() {
    return units::degree_t{GetAngle()};
}

frc::Pose2d DriveSubsystem::GetPose() {
    return odometry.GetPose();
}

float DriveSubsystem::Clamp(float input, float max) {
    if(max < input && 0 < input) {
        return max;
    }

    if(input < max && input < 0) {
        return -max;
    }

    return input;
}

void DriveSubsystem::ResetAngle() {
    pigeon.SetYaw(0);
}

void DriveSubsystem::SetSetpoint() {
    
}

void DriveSubsystem::ResetEncoders() {
    flEncoder.SetPosition(0);
    frEncoder.SetPosition(0);
    blEncoder.SetPosition(0);
    brEncoder.SetPosition(0);
}

double DriveSubsystem::GetAverageEncoder() {
    return (flEncoder.GetPosition() + frEncoder.GetPosition() + blEncoder.GetPosition() + brEncoder.GetPosition())/4;
}