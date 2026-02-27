// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc/XboxController.h>
#include <frc/controller/PIDController.h>
#include <frc/controller/ProfiledPIDController.h>
#include <frc/smartdashboard/SendableChooser.h>
#include <frc2/command/Command.h>
#include <frc2/command/InstantCommand.h>
#include <frc2/command/PIDCommand.h>
#include <frc2/command/ParallelRaceGroup.h>
#include <frc2/command/RunCommand.h>
#include <frc/DigitalInput.h>

#include "Constants.h"
#include "subsystems/DriveSubsystem.h"
#include "subsystems/Camera.h"
#include "subsystems/Shooter.h"
#include "subsystems/Turret.h"
#include "subsystems/LEDs.h"
#include "subsystems/Climber.h"
#include "subsystems/Intake.h"

/**
 * This class is where the bulk of the robot should be declared.  Since
 * Command-based is a "declarative" paradigm, very little robot logic should
 * actually be handled in the {@link Robot} periodic methods (other than the
 * scheduler calls).  Instead, the structure of the robot (including subsystems,
 * commands, and button mappings) should be declared here.
 */
class RobotContainer {
 public:
  RobotContainer();

  frc2::CommandPtr GetAutonomousCommand();
  void StopAll();

 private:
  // The driver's controller
  frc::XboxController m_driverController{OIConstants::kDriverControllerPort};
  frc::XboxController m_coDriverController{OIConstants::kCoDriverControllerPort};

  // The robot's subsystems and commands are defined here...

  // The robot's subsystems
  DriveSubsystem m_drive;
  CameraSubsystem m_camera;
  ShooterSubsystem m_shooter;
  TurretSubsystem m_turret;
  LEDSubsystem m_LEDs;
  ClimberSubsystem m_climber;
  IntakeSubsystem m_intake;

  // The chooser for the autonomous routines
  frc::SendableChooser<frc2::Command*> m_chooser;

  bool fieldRelative = false;

  int priorityTag = 0;

  //frc::DigitalInput m_ClimberLimitSwitch{1};

  /*frc::SlewRateLimiter<double> LSXLimiter{0.5_s};
  frc::SlewRateLimiter<double> LSYLimiter{0.5_s};
  frc::SlewRateLimiter<double> RSXLimiter{0.5_s};*/

  void ConfigureButtonBindings();
};
