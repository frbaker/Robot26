// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include "subsystems/DriveSubsystem.h"
#include "subsystems/Intake.h"
#include "subsystems/Shooter.h"
#include "subsystems/Turret.h"
#include "subsystems/Camera.h"
#include "subsystems/Climber.h"

/**
 * Registers named commands for PathPlanner event markers and auto sequences.
 * Must be called BEFORE AutoBuilder::configure() in RobotContainer.
 */
class AutoCommands {
 public:
  static void RegisterCommands(
      DriveSubsystem* drive,
      IntakeSubsystem* intake,
      ShooterSubsystem* shooter,
      TurretSubsystem* turret,
      CameraSubsystem* camera,
      ClimberSubsystem* climber);
};
