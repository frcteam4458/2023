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

#include <math.h>

#include <frc2/command/ConditionalCommand.h>

RobotContainer::RobotContainer() : 
driveSubsystem{},
extensionSubsystem{},
gripperSubsystem{},
pivotSubsystem{},

teleopCommand{&driveSubsystem},

station1{},
station2{},
station3{},
target{}

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

  frc::SmartDashboard::PutData("Floor Intake", new frc2::InstantCommand{
    [this] {
      pivotSubsystem.SetSetpoint(-12.9);
      extensionSubsystem.SetSetpoint(3.5);
      gripperSubsystem.SetSetpoint(-7.5);
    },
    {}
  });

  frc::SmartDashboard::PutData("Mid", new frc2::InstantCommand{
    [this] {
      pivotSubsystem.SetSetpoint(-35);
      extensionSubsystem.SetSetpoint(15);
    },
    {}
  });

  frc::SmartDashboard::PutData("Single Station", new frc2::InstantCommand{
    [this] {
      pivotSubsystem.SetSetpoint(-44);
      extensionSubsystem.SetSetpoint(4);
      gripperSubsystem.SetSetpoint(-7.5);
    },
    {}
  });

  frc::SmartDashboard::PutData("Double Station", new frc2::InstantCommand{
    [this] {
      pivotSubsystem.SetSetpoint(-51);
      extensionSubsystem.SetSetpoint(6);
      gripperSubsystem.SetSetpoint(-7.5);
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

  SetupAuto();

  frc::SmartDashboard::PutData(&chooser);

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

  return chooser.GetSelected();
}



void RobotContainer::SetupAuto() {
   chooser.AddOption("Blue 3 / Red 3", new frc2::SequentialCommandGroup{

    frc2::InstantCommand{[this] {
      driveSubsystem.ResetAngle();
    }},
    frc2::InstantCommand{
      [this] {
        extensionSubsystem.SetSoftLimits(false);
        extensionSubsystem.GetPID().SetOutputRange(-0.25, 0.25);
        // extensionSubsystem.SetSetpoint(5);
      }
    },
    frc2::WaitCommand{0.25_s},
    frc2::InstantCommand{
      [this] {
        pivotSubsystem.GetPID().SetOutputRange(-0.15, 0.5);
        // extensionSubsystem.GetMotor()->SetClosedLoopRampRate(1);
      },
      {}
    },
    frc2::InstantCommand{[this] {
      // pivotSubsystem.SetSetpoint(-15);
    }},
    frc2::WaitCommand{0.5_s},
    frc2::InstantCommand{[this] {
      pivotSubsystem.GetPID().SetOutputRange(-0.5, 0.5);
      // extensionSubsystem.SetSetpoint(-4.5);
    },
    {}},
    frc2::WaitCommand{0.5_s},
    frc2::InstantCommand{[this] {
      // gripperSubsystem.Set(-0.2);
    }},

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{[this] {
      // extensionSubsystem.SetSetpoint(5);
    }},

    frc2::WaitCommand{0.5_s},

    frc2::InstantCommand{[this] {
      // pivotSubsystem.SetSetpoint(-46);
    }},

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{[this] {
            extensionSubsystem.GetPID().SetOutputRange(-0.5, 0.33);
      // extensionSubsystem.SetSetpoint(-30);
    }},

    frc2::WaitCommand{1.5_s},

    frc2::InstantCommand{[this] {
      // gripperSubsystem.Set(0.2);
    }},

    frc2::InstantCommand{[this] {
      // extensionSubsystem.SetSetpoint(5);
      driveSubsystem.Drive(-0.1, 0);
    }},
    
    frc2::WaitCommand{0.5_s},

    frc2::InstantCommand{[this] {
      // pivotSubsystem.SetSetpoint(-13);
      driveSubsystem.Drive(0, 0);
    }},
    frc2::WaitCommand{0.5_s},
    frc2::InstantCommand{[this] {
      driveSubsystem.Drive(0, 0);
    }},
    frc2::ParallelRaceGroup{
      frc2::PIDCommand{
        frc::PIDController{0.005, 0.0, 0.0},
        [this] {
          return driveSubsystem.GetAngle();
        },
        150.0,
        [this](double output) {
          driveSubsystem.Drive(0.0, std::clamp(-output, -0.0833, 0.0833));
        }
      },
      frc2::WaitCommand{1.5_s},
    },

    frc2::InstantCommand{[this] {
      driveSubsystem.Drive(0, 0);
      // driveSubsystem.ResetEncoders();
    }},

    frc2::ParallelRaceGroup{
      frc2::PIDCommand{
        frc::PIDController{0.05, 0.0, 0.0},
        [this] {
          return driveSubsystem.GetAverageEncoder();
        },
        17.5,
        [this](double output) {
          driveSubsystem.Drive(std::clamp(output, -0.25, 0.25), 0);
        }
      },

      frc2::WaitCommand{1.5_s}
    },

    frc2::InstantCommand{[this] {
      driveSubsystem.Drive(0, 0);
    }},


    frc2::ParallelRaceGroup{
      frc2::PIDCommand{
        frc::PIDController{0.005, 0.0, 0.0},
        [this] {
          return driveSubsystem.GetAngle();
        },
        185.0,
        [this](double output) {
          driveSubsystem.Drive(0.0, std::clamp(-output, -0.166, 0.166));
        }
      },

      frc2::WaitCommand{1_s}
    },

    frc2::InstantCommand{[this] {
      // driveSubsystem.ResetEncoders();
    }},

    frc2::ParallelRaceGroup{
      frc2::PIDCommand{
        frc::PIDController{0.01, 0.0, 0.0},
        [this] {
          return driveSubsystem.GetAverageEncoder();
        },
        130.0,
        [this](double output) {
          driveSubsystem.Drive(std::clamp(output, -0.25, 0.25), 0);
        }
      },

      frc2::WaitCommand{3.25_s}
    },

    frc2::InstantCommand{[this] {
      driveSubsystem.Drive(0, 0);
    }}
  });

  












  chooser.AddOption("Blue 3 / Red 3", new frc2::SequentialCommandGroup{

    frc2::InstantCommand{[this] {
      driveSubsystem.ResetAngle();
    }},
    frc2::InstantCommand{
      [this] {
        extensionSubsystem.SetSoftLimits(false);
        extensionSubsystem.GetPID().SetOutputRange(-0.25, 0.25);
        extensionSubsystem.SetSetpoint(5);
      }
    },
    frc2::WaitCommand{0.25_s},
    frc2::InstantCommand{
      [this] {
        pivotSubsystem.GetPID().SetOutputRange(-0.15, 0.5);
        // extensionSubsystem.GetMotor()->SetClosedLoopRampRate(1);
      },
      {}
    },
    frc2::InstantCommand{[this] {
      pivotSubsystem.SetSetpoint(-15);
    }},
    frc2::WaitCommand{0.5_s},
    frc2::InstantCommand{[this] {
      pivotSubsystem.GetPID().SetOutputRange(-0.5, 0.5);
      extensionSubsystem.SetSetpoint(-4.5);
    },
    {}},
    frc2::WaitCommand{0.5_s},
    frc2::InstantCommand{[this] {
      gripperSubsystem.Set(-0.2);
    }},

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{[this] {
      extensionSubsystem.SetSetpoint(5);
    }},

    frc2::WaitCommand{0.5_s},

    frc2::InstantCommand{[this] {
      pivotSubsystem.SetSetpoint(-46);
    }},

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{[this] {
            extensionSubsystem.GetPID().SetOutputRange(-0.5, 0.33);
      extensionSubsystem.SetSetpoint(-30);
    }},

    frc2::WaitCommand{1.5_s},

    frc2::InstantCommand{[this] {
      gripperSubsystem.Set(0.2);
    }},

    frc2::InstantCommand{[this] {
      extensionSubsystem.SetSetpoint(5);
      driveSubsystem.Drive(-0.1, 0);
    }},
    
    frc2::WaitCommand{0.5_s},

    frc2::InstantCommand{[this] {
      pivotSubsystem.SetSetpoint(-13);
      driveSubsystem.Drive(0, 0);
    }},
    frc2::WaitCommand{0.5_s},
    frc2::InstantCommand{[this] {
      driveSubsystem.Drive(0, 0);
    }},

    frc2::ParallelRaceGroup{
      frc2::PIDCommand{
        frc::PIDController{0.005, 0.0, 0.0},
        [this] {
          return driveSubsystem.GetAngle();
        },
        150.0,
        [this](double output) {
          driveSubsystem.Drive(0.0, std::clamp(-output, -0.0833, 0.0833));
        }
      },
      frc2::WaitCommand{1.5_s},
    },

    frc2::InstantCommand{[this] {
      driveSubsystem.Drive(0, 0);
      // driveSubsystem.ResetEncoders();
    }},

    frc2::ParallelRaceGroup{
      frc2::PIDCommand{
        frc::PIDController{0.05, 0.0, 0.0},
        [this] {
          return driveSubsystem.GetAverageEncoder();
        },
        17.5,
        [this](double output) {
          driveSubsystem.Drive(std::clamp(output, -0.25, 0.25), 0);
        }
      },

      frc2::WaitCommand{1.5_s}
    },

    frc2::InstantCommand{[this] {
      driveSubsystem.Drive(0, 0);
    }},


    frc2::ParallelRaceGroup{
      frc2::PIDCommand{
        frc::PIDController{0.005, 0.0, 0.0},
        [this] {
          return driveSubsystem.GetAngle();
        },
        185.0,
        [this](double output) {
          driveSubsystem.Drive(0.0, std::clamp(-output, -0.0833, 0.0833));
        }
      },

      frc2::WaitCommand{1_s}
    },

    frc2::InstantCommand{[this] {
      // driveSubsystem.ResetEncoders();
    }},

    frc2::ParallelRaceGroup{
      frc2::PIDCommand{
        frc::PIDController{0.01, 0.0, 0.0},
        [this] {
          return driveSubsystem.GetAverageEncoder();
        },
        130.0,
        [this](double output) {
          driveSubsystem.Drive(std::clamp(output, -0.25, 0.25), 0);
        }
      },

      frc2::WaitCommand{3.25_s}
    },

    frc2::InstantCommand{[this] {
      driveSubsystem.Drive(0, 0);
    }}
  });


  





  chooser.AddOption("Blue 2 / Red 2", new frc2::SequentialCommandGroup{

    frc2::InstantCommand{[this] {
      driveSubsystem.ResetAngle();
    }},
    frc2::InstantCommand{
      [this] {
        extensionSubsystem.SetSoftLimits(false);
        extensionSubsystem.GetPID().SetOutputRange(-0.25, 0.25);
        // extensionSubsystem.SetSetpoint(5);
      }
    },
    frc2::WaitCommand{0.25_s},
    frc2::InstantCommand{
      [this] {
        pivotSubsystem.GetPID().SetOutputRange(-0.15, 0.5);
        // extensionSubsystem.GetMotor()->SetClosedLoopRampRate(1);
      },
      {}
    },
    frc2::InstantCommand{[this] {
      // pivotSubsystem.SetSetpoint(-15);
    }},
    frc2::WaitCommand{0.5_s},
    frc2::InstantCommand{[this] {
      pivotSubsystem.GetPID().SetOutputRange(-0.5, 0.5);
      // extensionSubsystem.SetSetpoint(-4.5);
    },
    {}},
    frc2::WaitCommand{0.5_s},
    frc2::InstantCommand{[this] {
      // gripperSubsystem.Set(-0.2);
    }},

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{[this] {
      // extensionSubsystem.SetSetpoint(5);
    }},

    frc2::WaitCommand{0.5_s},

    frc2::InstantCommand{[this] {
      // pivotSubsystem.SetSetpoint(-46);
    }},

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{[this] {
            extensionSubsystem.GetPID().SetOutputRange(-0.5, 0.33);
      // extensionSubsystem.SetSetpoint(-30);
    }},

    frc2::WaitCommand{1.5_s},

    frc2::InstantCommand{[this] {
      // gripperSubsystem.Set(0.2);
    }},

    frc2::InstantCommand{[this] {
      // extensionSubsystem.SetSetpoint(5);
      driveSubsystem.Drive(-0.1, 0);
    }},
    
    frc2::WaitCommand{0.5_s},

    frc2::InstantCommand{[this] {
      // pivotSubsystem.SetSetpoint(-13);
      driveSubsystem.Drive(0, 0);
      driveSubsystem.ResetEncoders();
    }},
    frc2::WaitCommand{0.5_s},

    frc2::ParallelRaceGroup{
      frc2::PIDCommand{
        frc::PIDController{0.05, 0.0, 0.0},
        [this] {
          frc::SmartDashboard::PutNumber("avg enc", driveSubsystem.GetAverageEncoder());
          return driveSubsystem.GetAverageEncoder();
        },
        -42.5,
        [this](double output) {
          driveSubsystem.Drive(std::clamp(output, -0.25, 0.25), 0);
        }
      }

    },

    frc2::InstantCommand{[this] {
      driveSubsystem.Drive(0, 0);
    }}}
  );
}