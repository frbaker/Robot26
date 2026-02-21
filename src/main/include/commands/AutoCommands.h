#pragma once

#include "subsystems/Shooter.h"
#include "subsystems/Turret.h"
#include "subsystems/Intake.h"
#include "subsystems/Camera.h"
#include "subsystems/Climber.h"

class AutoCommands {
 public:
  static void RegisterCommands(
      IntakeSubsystem* intake,
      ShooterSubsystem* shooter,
      TurretSubsystem* turret,
      CameraSubsystem* camera,
      ClimberSubsystem* climber);
};