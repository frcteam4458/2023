#pragma once

#ifndef DRIVE_SUBSYSTEM_H
#define DRIVE_SUBSYSTEM_H

#include <frc2/command/SubsystemBase.h>
#include "Constants.h"

#include <frc/motorcontrol/MotorControllerGroup.h>

#include <frc/motorcontrol/PWMSparkMax.h>
#include <frc/simulation/PWMSim.h>
#include <frc/Encoder.h>
#include <frc/simulation/EncoderSim.h>
#include <frc/simulation/DifferentialDrivetrainSim.h>

#include <frc/AnalogGyro.h>
#include <frc/simulation/AnalogGyroSim.h>

#include <frc/kinematics/DifferentialDriveKinematics.h>
#include <frc/kinematics/DifferentialDriveOdometry.h>
#include <frc/kinematics/DifferentialDriveWheelSpeeds.h>

#include <frc/drive/DifferentialDrive.h>

#include <frc/geometry/Rotation2d.h>
#include <frc/geometry/Pose2d.h>
#include <frc/geometry/Translation2d.h>

#include <units/angle.h>
#include <units/length.h>
#include <units/moment_of_inertia.h>
#include <units/mass.h>
#include <units/time.h>
#include <units/voltage.h>

#include <frc/smartdashboard/Field2d.h>

#include <frc/controller/SimpleMotorFeedforward.h>
#include <ctre/phoenix/sensors/PigeonIMU.h>
#include <ctre/phoenix/sensors/BasePigeonSimCollection.h>
#include <rev/CANSparkMax.h>


class DriveSubsystem : public frc2::SubsystemBase {
 public:
  DriveSubsystem();

  void Periodic() override;
  void SimulationPeriodic() override;
  void Drive(float fwd, float omega);
  void DriveVolts(units::volt_t left, units::volt_t right);
  void DriveSpeeds(units::meters_per_second_t left, units::meters_per_second_t right);
  void DriveCurvature(float fwd, float omega);
  float GetAngle();
  frc::DifferentialDriveWheelSpeeds GetWheelSpeeds();
  units::degree_t GetAngleDeg();
  frc::Pose2d GetPose();
  float Clamp(float input, float max);
  void ResetAngle();
  void SetSetpoint();

 private:

  #ifdef PWM_DRIVETRAIN
  frc::PWMSparkMax fl;
  frc::PWMSparkMax fr;
  frc::PWMSparkMax bl;
  frc::PWMSparkMax br;
  #endif

  #ifdef CAN_DRIVETRAIN
  rev::CANSparkMax fl;
  rev::CANSparkMax fr;
  rev::CANSparkMax bl;
  rev::CANSparkMax br;
  #endif

  frc::MotorControllerGroup leftMotors;
  frc::MotorControllerGroup rightMotors;

  frc::sim::PWMSim s_fl;
  frc::sim::PWMSim s_fr;
  frc::sim::PWMSim s_bl;
  frc::sim::PWMSim s_br;

  frc::Encoder left;
  frc::Encoder right;
  
  frc::sim::EncoderSim s_left;
  frc::sim::EncoderSim s_right;

  frc::AnalogGyro gyro;
  frc::sim::AnalogGyroSim s_gyro;

  // #ifdef __FRC_ROBORIO__
  ctre::phoenix::sensors::PigeonIMU pigeon;
  // #endif

  frc::DifferentialDriveKinematics kinematics;
  frc::DifferentialDriveOdometry odometry;

  frc::DifferentialDrive drive;

  frc::sim::DifferentialDrivetrainSim drivetrainSim;

  frc::Field2d field;

  frc::SimpleMotorFeedforward<units::meters> leftFF{kS, kV, kA};
  frc::SimpleMotorFeedforward<units::meters> rightFF{kS, kV, kA};
};

#endif