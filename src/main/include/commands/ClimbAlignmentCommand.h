// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#pragma once

#include <frc2/command/Command.h>
#include <frc2/command/CommandHelper.h>
#include <frc/Timer.h>
#include <frc/smartdashboard/SmartDashboard.h>

#include "subsystems/DriveSubsystem.h"
#include "subsystems/Camera.h"
#include "subsystems/Climber.h"
#include "utils/CollisionDetector.h"
#include "Constants.h"

/**
 * Multi-phase autonomous climb alignment command.
 *
 * Phase 1: Vision align to AprilTag on wall, position right of upright
 * Phase 2: Strafe left into upright (detect via current spike)
 * Phase 3: Back up until limit switch triggered
 *
 * Then: Stop and trigger climb motor
 */
class ClimbAlignmentCommand
    : public frc2::CommandHelper<frc2::Command, ClimbAlignmentCommand> {
 public:
  ClimbAlignmentCommand(DriveSubsystem* drive, CameraSubsystem* camera,
                        ClimberSubsystem* climber);

  void Initialize() override;
  void Execute() override;
  void End(bool interrupted) override;
  bool IsFinished() override;

 private:
  enum class Phase { kVisionAlign, kStrafeIntoUpright, kReverseToLimitSwitch, kDone };

  // Subsystems
  DriveSubsystem* m_drive;
  CameraSubsystem* m_camera;
  ClimberSubsystem* m_climber;

  // State
  Phase m_currentPhase = Phase::kVisionAlign;
  frc::Timer m_phaseTimer;
  double m_headingSnapshot = 0.0;  // For heading lock in phases 2 & 3
  int m_limitSwitchDebounceCount = 0;

  // Collision detector for phase 2
  CollisionDetector m_collisionDetector{
      ClimbAlignmentConstants::kCurrentThresholdAmps,
      ClimbAlignmentConstants::kCollisionConfirmTimeSeconds,
      ClimbAlignmentConstants::kVelocityDropRatio};

  // Phase execution methods
  void ExecutePhase1_VisionAlign();
  void ExecutePhase2_StrafeIntoUpright();
  void ExecutePhase3_ReverseToLimitSwitch();

  // Helper to get the correct ladder AprilTag based on alliance
  int GetLadderTag();

  // Logging helper
  void LogToDashboard();
};
