#include "RobotContainer.h"

#include <frc/smartdashboard/SmartDashboard.h>

#include <frc/trajectory/constraint/DifferentialDriveVoltageConstraint.h>

#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>

#include <frc2/command/SequentialCommandGroup.h>
#include <frc2/command/InstantCommand.h>
#include "Controls.h"
#include "Constants.h"

#include <math.h>

#include <frc2/command/PIDCommand.h>

#include <frc2/command/WaitUntilCommand.h>
#include <frc2/command/WaitCommand.h>

#include <frc2/command/CommandPtr.h>

#include <wpi/raw_ostream.h>

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

  leftBumper.WhileHeld(frc2::InstantCommand{
    [this] {
      gripperSubsystem.Set(0.5);
    },
    {&gripperSubsystem}
  });
  leftBumper.WhenReleased(frc2::InstantCommand{
    [this] {
      gripperSubsystem.Set(0);
    },
    {&gripperSubsystem}
  });
  rightBumper.WhileHeld(frc2::InstantCommand{
    [this] {
      gripperSubsystem.Set(-0.5);
    },
    {&gripperSubsystem}
  });
  rightBumper.WhenReleased(frc2::InstantCommand{
    [this] {
      gripperSubsystem.Set(0);
    },
    {&gripperSubsystem}
  });

  pivotSubsystem.SetDefaultCommand(
    frc2::RunCommand{
      [this] {
        double input = pivotExtensionStick.GetRawAxis(1);
        // if(std::abs(input) < 0.1) {
        //   input = 0.0;
        // }
        pivotSubsystem.Set(input/4);
      },
      {&pivotSubsystem}
    }
  );

  extensionSubsystem.SetDefaultCommand(
    frc2::RunCommand{
      [this] {
        double input = pivotExtensionStick.GetRawAxis(0);
        // if(std::abs(input) < 0.2) {
        //   input = 0.0;
        // }

        if(input < 0) input /= 4;
        else input /= 2;
        extensionSubsystem.Set(-input);
      },
      {&extensionSubsystem}
    }
  );

  toggleSoftLimits.WhenPressed(frc2::InstantCommand{
    [this] {
      extensionSubsystem.SetSoftLimits(!extensionSubsystem.GetSoftLimits());
    },
    {}
  });

  // button2.WhenPressed(frc2::InstantCommand{
  //   [this] {
  //     extensionSubsystem.SetSetpoint(0);
  //     pivotSubsystem.SetSetpoint(0);
  //     gripperSubsystem.SetSetpoint(0);
  //   },
  //   {

  //   }
  // });

  frc::SmartDashboard::PutData("Setpoint 1", new frc2::InstantCommand{
    [this] {
      extensionSubsystem.SetSetpoint(40);
      // pivotSubsystem.SetSetpoint(0);
      // gripperSubsystem.SetSetpoint(0);
    },
    {}
  });

  frc::SmartDashboard::PutData("Setpoint 2", new frc2::InstantCommand{
    [this] {
      extensionSubsystem.SetSetpoint(0);
      pivotSubsystem.SetSetpoint(0);
      gripperSubsystem.SetSetpoint(0);
    },
    {}
  });

  frc::SmartDashboard::PutData("Setpoint 3", new frc2::InstantCommand{
    [this] {
      extensionSubsystem.SetSetpoint(0);
      pivotSubsystem.SetSetpoint(0);
      gripperSubsystem.SetSetpoint(0);
    },
    {}
  });

  frc::SmartDashboard::PutData("Setpoint 4", new frc2::InstantCommand{
    [this] {
      extensionSubsystem.SetSetpoint(0);
      pivotSubsystem.SetSetpoint(0);
      gripperSubsystem.SetSetpoint(0);
    },
    {}
  });

  frc::SmartDashboard::PutData("Setpoint 5", new frc2::InstantCommand{
    [this] {
      extensionSubsystem.SetSetpoint(0);
      pivotSubsystem.SetSetpoint(0);
      gripperSubsystem.SetSetpoint(0);
    },
    {}
  });

  frc::SmartDashboard::PutData("Setpoint 6", new frc2::InstantCommand{
    [this] {
      extensionSubsystem.SetSetpoint(0);
      pivotSubsystem.SetSetpoint(0);
      gripperSubsystem.SetSetpoint(0);
    },
    {}
  });

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



  return new frc2::SequentialCommandGroup{
    frc2::InstantCommand{
      [this] {
        extensionSubsystem.GetMotor()->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, false);
        extensionSubsystem.Set(0.25);
      }
    },

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{
      [this] {
        extensionSubsystem.GetEncoder().SetPosition(0);
        extensionSubsystem.GetMotor()->EnableSoftLimit(rev::CANSparkMax::SoftLimitDirection::kForward, true);
      }
    },
    frc2::InstantCommand{
      [this] {
        pivotSubsystem.GetPID().SetOutputRange(-0.33, 0.33);
        // extensionSubsystem.GetMotor()->SetClosedLoopRampRate(1);
        extensionSubsystem.GetPID().SetOutputRange(-0.5, 0.5);
        extensionSubsystem.SetSetpoint(1);

      },
      {}
    },
    frc2::WaitCommand{1_s},
    frc2::InstantCommand{[this] {
      pivotSubsystem.SetSetpoint(-15);
    },
    {}},
    frc2::WaitCommand{1_s},
    frc2::InstantCommand{[this] {
      extensionSubsystem.SetSetpoint(-6);
    },
    {}},
    frc2::WaitCommand{1_s},
    frc2::InstantCommand{[this] {
      gripperSubsystem.Set(-0.2);
    }},

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{[this] {
      extensionSubsystem.SetSetpoint(0);
    }},

    frc2::InstantCommand{[this] {
      pivotSubsystem.SetSetpoint(-46);
    }},

    frc2::WaitCommand{2_s},

    frc2::InstantCommand{[this] {
      extensionSubsystem.SetSetpoint(-42);
    }},

    frc2::WaitCommand{2_s},

    frc2::InstantCommand{[this] {
      gripperSubsystem.Set(0.2);
    }},

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{[this] {
      gripperSubsystem.Set(0);
      extensionSubsystem.SetSetpoint(0);
    }},
    
    frc2::WaitCommand{0.5_s},

    frc2::InstantCommand{[this] {
      pivotSubsystem.SetSetpoint(0);
    }},

    frc2::PIDCommand{
      frc::PIDController{0.5, 0.0, 0.0},
      [this] {
        return driveSubsystem.GetPose().Translation().X().value();
      },
      5.0,
      [this](double output) {
        driveSubsystem.Drive(output, 0);
      },
      {&driveSubsystem}
    }
  };

  // return new frc2::SequentialCommandGroup{
  //   frc2::InstantCommand{
  //     [this] {
  //       extensionSubsystem.SetSetpoint(40);

  //     },
  //     {}
  //   },
  //   frc2::InstantCommand{[this] {
  //     while(!extensionSubsystem.AtSetpoint());
  //   },
  //   {}},
  //   frc2::InstantCommand{
  //     [this] {
  //       pivotSubsystem.SetSetpoint(-20);
  //     },
  //     {}
  //   },
  //   frc2::WaitUntilCommand{[this] {
  //     return pivotSubsystem.AtSetpoint();
  //   }},
  //   frc2::InstantCommand{
  //     [this] {
  //       gripperSubsystem.Set(0.5);
  //     }
  //   },
  //   frc2::WaitCommand{1.5_s},
  //   frc2::InstantCommand{
  //     [this] {
  //       pivotSubsystem.SetSetpoint(-51);
  //     },
  //     {}
  //   },
  //   frc2::WaitUntilCommand{[this] {
  //     return pivotSubsystem.AtSetpoint();
  //   }},
  //   frc2::InstantCommand{
  //     [this] {
  //       extensionSubsystem.SetSetpoint(0);
  //     },
  //     {}
  //   },
  //   frc2::WaitUntilCommand{
  //     [this] {
  //       return extensionSubsystem.AtSetpoint();
  //     }
  //   },
  //   frc2::WaitCommand{1_s},
  //   frc2::InstantCommand{
  //     [this] {
  //       gripperSubsystem.Set(-0.5);
  //     }
  //   },
  //   frc2::WaitCommand{0.25_s},
  //   frc2::InstantCommand{
  //     [this] {
  //       gripperSubsystem.Set(0);
  //     }
  //   },
  //   frc2::ParallelRaceGroup{
  //     frc2::PIDCommand{
  //       frc::PIDController{0.5, 0.0, 0.0},
  //       [this] {
  //         return driveSubsystem.GetPose().X().value();
  //       },
  //       10.0,
  //       [this](double output) {
  //         driveSubsystem.Drive(-output, 0);
  //       },
  //       {&driveSubsystem}
  //     },
  //     frc2::WaitCommand{5.0_s}
  //   },
  //   frc2::InstantCommand{
  //     [this] {
        
  //     },
  //     {}
  //   }
  // };

  // command.Schedule();
  // return new frc2::SequentialCommandGroup{
  //   // frc2::InstantCommand{
  //   //   [this] {
  //   //     driveSubsystem.ResetAngle();
  //   //   }
  //   // }
  //   std::move(ramseteCommand),
  //   frc2::InstantCommand{
  //     [this] {
  //       driveSubsystem.DriveVolts(0_V, 0_V);
  //     }
  //   }
  // };


}