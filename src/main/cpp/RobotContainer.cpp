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

#include <frc2/command/Command.h>
#include <frc2/command/Commands.h>

#include <frc/shuffleboard/Shuffleboard.h>

#include "Setpoints.h"

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
    frc::SmartDashboard::PutNumber("Auto Distance", -44.5);

}


void RobotContainer::ConfigureButtonBindings() {
  frc::SmartDashboard::PutBoolean("Gripper", false);
    frc::SmartDashboard::PutNumber("Auto Distance", -44.5);

  leftBumper.WhenPressed(frc2::InstantCommand{
    [this] {
      gripperSubsystem.Set(GRIPPER_POWER);
      frc::SmartDashboard::PutBoolean("Gripper", false);
    }
  });

  rightBumper.WhenPressed(frc2::InstantCommand{
    [this] {
      gripperSubsystem.Set(-GRIPPER_POWER);
      frc::SmartDashboard::PutBoolean("Gripper", true);
    }
  });

  // TODO add Operator gripper control

  pivotSubsystem.SetDefaultCommand(
    frc2::RunCommand{
      [this] {
        double input = pivotExtensionStick.GetRawAxis(1);
        if(std::abs(input) < DEADZONE) {
          input = 0;
        }
        pivotSubsystem.AddSetpoint(input/4);
      },
      {&pivotSubsystem}
    }
  );

  extensionSubsystem.SetDefaultCommand(
    frc2::RunCommand{
      [this] {
        double input = pivotExtensionStick.GetRawAxis(0);
        if(std::abs(input) < DEADZONE) {
          input = 0;
        }
        extensionSubsystem.AddSetpoint(-input/4);
      },
      {&extensionSubsystem}
    }
  );

  toggleSoftLimits.WhenPressed(frc2::InstantCommand{
    [this] {
      extensionSubsystem.SetSoftLimits(!extensionSubsystem.GetSoftLimits());
    }
  });


  intake.WhenPressed(frc2::InstantCommand{
    [this] {
      pivotSubsystem.SetSetpoint(FLOOR_INTAKE[0]);
      extensionSubsystem.SetSetpoint(FLOOR_INTAKE[1]);
      // gripperSubsystem.SetSetpoint(FLOOR_INTAKE[2]);
    }
  });

  readyStick.WhenPressed(frc2::InstantCommand{
    [this] {
      pivotSubsystem.SetSetpoint(READY[0]);
      extensionSubsystem.SetSetpoint(READY[1]);
    }
  });

  ready.WhenPressed(frc2::InstantCommand{
    [this] {
      pivotSubsystem.SetSetpoint(READY[0]);
      extensionSubsystem.SetSetpoint(READY[1]);
    }
  });



  single.WhenPressed(frc2::InstantCommand{
    [this] {
      pivotSubsystem.SetSetpoint(SINGLE_STATION[0]);
      extensionSubsystem.SetSetpoint(SINGLE_STATION[1]);
      // gripperSubsystem.SetSetpoint(SINGLE_STATION[2]);
    }
  });

  singlestick.WhenPressed(frc2::InstantCommand{
    [this] {
      pivotSubsystem.SetSetpoint(SINGLE_STATION[0]);
      extensionSubsystem.SetSetpoint(SINGLE_STATION[1]);
      // gripperSubsystem.SetSetpoint(SINGLE_STATION[2]);
    }
  });

  slidestick.WhenPressed(frc2::InstantCommand{
    [this] {
      pivotSubsystem.SetSetpoint(DOUBLE_STATION[0]);
      extensionSubsystem.SetSetpoint(DOUBLE_STATION[1]);
      // gripperSubsystem.SetSetpoint(DOUBLE_STATION[2]);
    }
  });

  
  doubleb.WhenPressed(frc2::InstantCommand{
    [this] {
      pivotSubsystem.SetSetpoint(DOUBLE_STATION[0]);
      extensionSubsystem.SetSetpoint(DOUBLE_STATION[1]);
      // gripperSubsystem.SetSetpoint(DOUBLE_STATION[2]);
    }
  });

  team.WhenPressed(frc2::InstantCommand{
    [this] {
      alliance = frc::DriverStation::GetAlliance();
      for(int i = 0; i < 60; i++) {
        if(alliance == frc::DriverStation::Alliance::kRed)
        buffer[i].SetRGB(127, 0, 0);
        else if(alliance == frc::DriverStation::Alliance::kBlue)
        buffer[i].SetRGB(0, 0, 127);
        else
        buffer[i].SetRGB(127, 0, 127);
      }

      GetLEDStrip()->SetData(buffer);
      GetLEDStrip()->Start();
    }
  });

  orange.WhenPressed(frc2::InstantCommand{
    [this] {
      for(int i = 0; i < 60; i++) {
        buffer[i].SetRGB(127, 45, 0);
      }
      GetLEDStrip()->SetData(buffer);
      GetLEDStrip()->Start();
    }
  });

  purple.WhenPressed(frc2::InstantCommand{
    [this] {
      for(int i = 0; i < 60; i++) {
        buffer[i].SetRGB(127, 0, 127);
      }
      GetLEDStrip()->SetData(buffer);
      GetLEDStrip()->Start();
    }
  });


  // this is gonna need more work

  frc::SmartDashboard::PutData("(Ready) Score Mid", new frc2::SequentialCommandGroup{
    frc2::InstantCommand{[this] {
      pivotSubsystem.SetSetpoint(-45);
    }},

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{[this] {
      extensionSubsystem.SetSetpoint(-17);
    }},

    frc2::WaitCommand{0.75_s},

    frc2::InstantCommand{[this] {
      gripperSubsystem.Set(0.5);
    }},

    frc2::WaitCommand{0.25_s},

    frc2::InstantCommand{[this] {
      extensionSubsystem.SetSetpoint(3);
    }},

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{[this] {
      pivotSubsystem.SetSetpoint(-8);
    }}
  });

  frc::SmartDashboard::PutData("(Ready) Score High", new frc2::SequentialCommandGroup{
    frc2::InstantCommand{[this] {
      pivotSubsystem.SetSetpoint(-46);
    }},

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{[this] {
      extensionSubsystem.SetSetpoint(-30);
    }},

    frc2::WaitCommand{1.25_s},

    frc2::InstantCommand{[this] {
      gripperSubsystem.Set(0.5);
    }},

    frc2::WaitCommand{0.25_s},

    frc2::InstantCommand{[this] {
      extensionSubsystem.SetSetpoint(3);
    }},

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{[this] {
      pivotSubsystem.SetSetpoint(-8);
    }}
  });

  frc::SmartDashboard::PutData("Mid", new frc2::SequentialCommandGroup{
    frc2::InstantCommand{
      [this] {
        pivotSubsystem.SetSetpoint(-45);
      }
    },

    frc2::WaitCommand{1_s},

    frc2::InstantCommand{
      [this] {
        extensionSubsystem.SetSetpoint(-20);
      }
    }
  });

  frc::SmartDashboard::PutData("High", new frc2::SequentialCommandGroup{
    frc2::InstantCommand{
      [this] {
        pivotSubsystem.SetSetpoint(-50);
      }
    },

    frc2::WaitCommand{1.25_s},

    frc2::InstantCommand{
      [this] {
        extensionSubsystem.SetSetpoint(-30);
      }
    }
  });

  // gripper operator controls

  frc::SmartDashboard::PutData("Gripper Close", new frc2::InstantCommand{[this] {
    gripperSubsystem.Set(-0.5);
  }});

  frc::SmartDashboard::PutData("Gripper Open", new frc2::InstantCommand{[this] {
    gripperSubsystem.Set(0.5);
  }});


  frc::SmartDashboard::PutNumber("Auto Distance", -44.5);
  SetupAuto();
  frc::SmartDashboard::PutNumber("Auto Distance", -44.5);


  frc::SmartDashboard::PutData("Autonomous Selector", &chooser);

}

frc2::Command* RobotContainer::GetTeleopCommand() {
  return &teleopCommand;
}

frc2::CommandPtr RobotContainer::GetAutonomousCommand() {
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



  if(chooser.GetSelected() == 1) { // blue 1 (UNTESTED)
    return frc2::cmd::Sequence(
    AutoScoreRoutine(),
    
    frc2::PIDCommand{
      frc::PIDController{0.005, 0.0, 0.0},
      [this] {
        return driveSubsystem.GetAngle();
      },
      -150.0,
      [this](double output) {
        driveSubsystem.Drive(0.0, std::clamp(output, -0.0833, 0.0833));
      }
    }.WithTimeout(1.5_s).AndThen([this] { driveSubsystem.Drive(0, 0); driveSubsystem.ResetEncoders(); }),

    frc2::PIDCommand{
      frc::PIDController{0.05, 0.0, 0.0},
      [this] {
        return driveSubsystem.GetAverageEncoder();
      },
      17.5,
      [this](double output) {
        driveSubsystem.Drive(std::clamp(output, -0.25, 0.25), 0);
      }
    }.WithTimeout(1.5_s).AndThen([this] { driveSubsystem.Drive(0, 0); driveSubsystem.ResetEncoders(); }),

    frc2::PIDCommand{
      frc::PIDController{0.005, 0.0, 0.0},
      [this] {
        return driveSubsystem.GetAngle();
      },
      -185.0,
      [this](double output) {
        driveSubsystem.Drive(0.0, std::clamp(output, -0.166, 0.166));
      }
    }.WithTimeout(1_s).AndThen([this] { driveSubsystem.Drive(0, 0); driveSubsystem.ResetEncoders(); }),

    frc2::PIDCommand{
      frc::PIDController{0.01, 0.0, 0.0},
      [this] {
        return driveSubsystem.GetAverageEncoder();
      },
      130.0,
      [this](double output) {
        driveSubsystem.Drive(std::clamp(output, -0.25, 0.25), 0);
      }
    }.WithTimeout(3.25_s).AndThen([this] { driveSubsystem.Drive(0, 0); driveSubsystem.ResetEncoders(); })
   );
  }



  if(chooser.GetSelected() == 2) { // balance (WORKING)
    return frc2::cmd::Sequence(
    AutoScoreRoutine(),

    frc2::cmd::RunOnce([this] {
      driveSubsystem.ResetEncoders();
    }),
    
    frc2::PIDCommand{
      frc::PIDController{0.7, 0.0, 0.0},
      [this] {
        return driveSubsystem.GetAverageEncoder();
      },
      -48.5,
      [this](double output) {
        if(output > 0.25 && output > 0) {
          output = 0.25;
        }

        if(output < -0.25 && output < 0) {
          output = -0.25;
        }
        driveSubsystem.Drive(output, 0);
      }
    }
   ); 
  }



  if(chooser.GetSelected() == 3) { // blue 3 (WORKING)
    return frc2::cmd::Sequence(
    AutoScoreRoutine(),
    
    frc2::PIDCommand{
      frc::PIDController{0.005, 0.0, 0.0},
      [this] {
        return driveSubsystem.GetAngle();
      },
      150.0,
      [this](double output) {
        driveSubsystem.Drive(0.0, std::clamp(-output, -0.0833, 0.0833));
      }
    }.WithTimeout(1.5_s).AndThen([this] { driveSubsystem.Drive(0, 0); driveSubsystem.ResetEncoders(); }),

    frc2::PIDCommand{
      frc::PIDController{0.05, 0.0, 0.0},
      [this] {
        return driveSubsystem.GetAverageEncoder();
      },
      17.5,
      [this](double output) {
        driveSubsystem.Drive(std::clamp(output, -0.25, 0.25), 0);
      }
    }.WithTimeout(1.5_s).AndThen([this] { driveSubsystem.Drive(0, 0); driveSubsystem.ResetEncoders(); }),

    frc2::PIDCommand{
      frc::PIDController{0.005, 0.0, 0.0},
      [this] {
        return driveSubsystem.GetAngle();
      },
      185.0,
      [this](double output) {
        driveSubsystem.Drive(0.0, std::clamp(-output, -0.166, 0.166));
      }
    }.WithTimeout(1_s).AndThen([this] { driveSubsystem.Drive(0, 0); driveSubsystem.ResetEncoders(); }),

    frc2::PIDCommand{
      frc::PIDController{0.01, 0.0, 0.0},
      [this] {
        return driveSubsystem.GetAverageEncoder();
      },
      130.0,
      [this](double output) {
        driveSubsystem.Drive(std::clamp(output, -0.25, 0.25), 0);
      }
    }.WithTimeout(3.25_s).AndThen([this] { driveSubsystem.Drive(0, 0); driveSubsystem.ResetEncoders(); })
   );
  }

  if(chooser.GetSelected() == 4) { // RED 3 (UNTESTED)
    return frc2::cmd::Sequence(
      AutoScoreRoutine()
    );
  }

  if(chooser.GetSelected() == 5) { // RED 1 (UNTESTED)
    return frc2::cmd::Sequence(
      AutoScoreRoutine(),

      frc2::InstantCommand{[this] {
        driveSubsystem.Drive(-0.25, 0);
      }},

      frc2::WaitCommand{3_s},

      frc2::PIDCommand{
        frc::PIDController{0.05, 0.0, 0.0},
        [this] {
          return driveSubsystem.GetPitch();
        },
        0.0,
        [this](double output) {
          driveSubsystem.Drive(std::clamp(output, -0.25, 0.25), 0);
        }
      }
    );
  }

  if(chooser.GetSelected() == 6) { // middle no balance
    return frc2::cmd::Sequence(
    AutoScoreRoutine()
   ); 
  }
}



void RobotContainer::SetupAuto() {
  chooser.AddOption("Blue 1", 1); // long
  chooser.SetDefaultOption("Real Balance", 2);
  chooser.AddOption("Blue 3", 3); // short
  chooser.AddOption("Red 3", 4); // long
  chooser.AddOption("Red 1", 5); // short
  chooser.AddOption("Middle No Balance", 6);
}

frc2::CommandPtr RobotContainer::AutoScoreRoutine() {
  return frc2::cmd::Sequence(
    frc2::cmd::RunOnce([this] {
      driveSubsystem.ResetAngle();
      extensionSubsystem.SetSoftLimits(false);
      extensionSubsystem.GetPID().SetOutputRange(-0.25, 0.25);
      extensionSubsystem.SetPIDRange(-0.25, 0.25);
      extensionSubsystem.SetSetpoint(5);
      pivotSubsystem.GetPID().SetOutputRange(-0.15, 0.5);
      pivotSubsystem.SetPIDRange(-0.15, 0.5);
    }),

    frc2::cmd::Wait(0.25_s),

    frc2::cmd::RunOnce([this] {
      pivotSubsystem.SetSetpoint(-15);
    }),

    frc2::cmd::Wait(0.5_s),

    frc2::cmd::RunOnce([this] {
      pivotSubsystem.GetPID().SetOutputRange(-0.5, 0.5);
      pivotSubsystem.SetPIDRange(-0.5, 0.5);
      extensionSubsystem.SetSetpoint(-4.5);
    }),

    frc2::cmd::Wait(0.5_s),

    frc2::cmd::RunOnce([this] {
      gripperSubsystem.Set(-0.2);
    }),

    frc2::cmd::Wait(1_s),

    frc2::cmd::RunOnce([this] {
      extensionSubsystem.SetSetpoint(5);
    }),

    frc2::cmd::Wait(0.5_s),

    frc2::cmd::RunOnce([this] {
      pivotSubsystem.SetSetpoint(-46);
    }),

    frc2::cmd::Wait(1_s),

    frc2::cmd::RunOnce([this] {
      extensionSubsystem.GetPID().SetOutputRange(-0.5, 0.33);
      extensionSubsystem.SetPIDRange(-0.5, 0.33);
      extensionSubsystem.SetSetpoint(-30);
    }),

    frc2::cmd::Wait(1.5_s),

    frc2::cmd::RunOnce([this] {
      gripperSubsystem.Set(0.2);
    }),

    frc2::cmd::Wait(0.5_s),

    frc2::cmd::RunOnce([this] {
      extensionSubsystem.SetSetpoint(5);
      driveSubsystem.Drive(-0.1, 0);
    }),
    
    frc2::cmd::Wait(0.5_s),

    frc2::cmd::RunOnce([this] {
      pivotSubsystem.SetSetpoint(0);
      driveSubsystem.Drive(0, 0);
      pivotSubsystem.GetPID().SetOutputRange(-0.4, 0.5);
      pivotSubsystem.SetPIDRange(-0.4, 0.5);
    })
  );

}

frc::AddressableLED* RobotContainer::GetLEDStrip() {
  return &led;
}