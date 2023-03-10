#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>

#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include "Controls.h"
#include "Constants.h"


RobotContainer::RobotContainer() : 
driveSubsystem{},
extensionSubsystem{},
gripperSubsystem{},
pivotSubsystem{},

teleopCommand{&driveSubsystem}

{
  ConfigureButtonBindings();
}

void RobotContainer::ConfigureButtonBindings() {

  button1.OnTrue(
    frc2::InstantCommand{
      [this] {
        if(GripCommand::GetClosed()) {
          gripperSubsystem.GetCurrentCommand()->Cancel();
        }
      }
    }.ToPtr()
  );

  button1.OnTrue(
    frc2::InstantCommand{
      [this] {
        if(GripCommand::GetClosed()) {
          gripperSubsystem.GetCurrentCommand()->Cancel();
        }
        std::move(GripCommand{&gripperSubsystem, CONE_AMPERAGE}).Schedule();
      }
    }.ToPtr()
  );

  button2.OnTrue(
    frc2::InstantCommand{
      [this] {
        if(GripCommand::GetClosed()) {
          gripperSubsystem.GetCurrentCommand()->Cancel();
        }
        std::move(GripCommand{&gripperSubsystem, CUBE_AMPERAGE}).Schedule();
      }
    }.ToPtr()
  );

  pivotSubsystem.SetDefaultCommand(
    frc2::RunCommand{
      [this] {
        pivotSubsystem.Set(pivotExtensionStick.GetRawAxis(1));
      },
      {&pivotSubsystem}
    }
  );

  extensionSubsystem.SetDefaultCommand(
    frc2::RunCommand{
      [this] {
        extensionSubsystem.Set(pivotExtensionStick.GetRawAxis(0));
      },
      {&extensionSubsystem}
    }
  );
}

frc2::Command* RobotContainer::GetTeleopCommand() {
  return &teleopCommand;
}

frc2::Command* RobotContainer::GetAutonomousCommand() {

  frc::TrajectoryConfig trajectoryConfig{0.5_m / 1_s, 0.5_m / 1_s / 1_s};

  frc::Trajectory trajectory = frc::TrajectoryGenerator::GenerateTrajectory(
    frc::Pose2d{0_m, 0_m, frc::Rotation2d{0_deg}},
    {
      frc::Translation2d{2_m, 0_m},
      frc::Translation2d{4.5_m, 0_m}

    },
    frc::Pose2d{4.5_m, 10_m, frc::Rotation2d{90_deg}},

    trajectoryConfig
  );
  
  frc2::RamseteCommand ramseteCommand{
    trajectory,
    [this]() {
      
      return driveSubsystem.GetPose();
    },
    frc::RamseteController{
      units::unit_t<frc::RamseteController::b_unit>{2.0},
      units::unit_t<frc::RamseteController::zeta_unit>{0.7}
    },
    frc::SimpleMotorFeedforward<units::meters>{kS, kV, kA},
    frc::DifferentialDriveKinematics{trackWidth},
    [this]() {
      return driveSubsystem.GetWheelSpeeds();
    },
    frc2::PIDController{akP, 0, 0},
    frc2::PIDController{akP, 0, 0},
    [this](auto left, auto right) {
      driveSubsystem.DriveVolts(left, right);
    },
    {
      &driveSubsystem
    }
  };

  driveSubsystem.ResetAngle();
  return new frc2::SequentialCommandGroup{
    // frc2::InstantCommand{
    //   [this] {
    //     driveSubsystem.ResetAngle();
    //   }
    // }
    std::move(ramseteCommand),
    frc2::InstantCommand{
      [this] {
        driveSubsystem.DriveVolts(0_V, 0_V);
      }
    }
  };
}
