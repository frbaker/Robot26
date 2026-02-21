// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/AutoCommands.h"

#include <frc/DriverStation.h>
#include <frc2/command/Commands.h>
#include <frc2/command/FunctionalCommand.h>
#include <frc2/command/WaitCommand.h>
#include <pathplanner/lib/auto/NamedCommands.h>
#include <cmath>

#include "Constants.h"
#include "commands/ClimbAlignmentCommand.h"

namespace {
// Helper functions for alliance-specific tags
int GetHubAllianceTag1() {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value() &&
      alliance.value() == frc::DriverStation::Alliance::kRed) {
    return AprilTagConstants::Red::kHubAllianceSide1;
  }
  return AprilTagConstants::Blue::kHubAllianceSide1;
}

int GetHubAllianceTag2() {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value() &&
      alliance.value() == frc::DriverStation::Alliance::kRed) {
    return AprilTagConstants::Red::kHubAllianceSide2;
  }
  return AprilTagConstants::Blue::kHubAllianceSide2;
}

int GetLadderTag1() {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value() &&
      alliance.value() == frc::DriverStation::Alliance::kRed) {
    return AprilTagConstants::Red::kLadder1;
  }
  return AprilTagConstants::Blue::kLadder1;
}

int GetLadderTag2() {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value() &&
      alliance.value() == frc::DriverStation::Alliance::kRed) {
    return AprilTagConstants::Red::kLadder2;
  }
  return AprilTagConstants::Blue::kLadder2;
}

int GetHubNeutralTag1() {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value() &&
      alliance.value() == frc::DriverStation::Alliance::kRed) {
    return AprilTagConstants::Red::kHubNeutralSide1;  // Tag 3
  }
  return AprilTagConstants::Blue::kHubNeutralSide1;  // Tag 19
}

int GetHubNeutralTag2() {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value() &&
      alliance.value() == frc::DriverStation::Alliance::kRed) {
    return AprilTagConstants::Red::kHubNeutralSide2;  // Tag 4
  }
  return AprilTagConstants::Blue::kHubNeutralSide2;  // Tag 20
}

bool IsRedAlliance() {
  auto alliance = frc::DriverStation::GetAlliance();
  return alliance.has_value() &&
         alliance.value() == frc::DriverStation::Alliance::kRed;
}

// Offset angle for neutral zone shooting (degrees)
constexpr double kNeutralZoneOffset = 20.0;
}  // namespace

void AutoCommands::RegisterCommands(DriveSubsystem* drive,
                                    IntakeSubsystem* intake,
                                    ShooterSubsystem* shooter,
                                    TurretSubsystem* turret,
                                    CameraSubsystem* camera,
                                    ClimberSubsystem* climber) {
  using namespace pathplanner;

  // ========== INTAKE COMMANDS ==========
  NamedCommands::registerCommand(
      "IntakeStart",
      frc2::cmd::RunOnce([intake] { intake->Run(); }, {intake}));

  NamedCommands::registerCommand(
      "IntakeStop",
      frc2::cmd::RunOnce([intake] { intake->Stop(); }, {intake}));

  NamedCommands::registerCommand(
      "IntakeReverse",
      frc2::cmd::RunOnce([intake] { intake->Reverse(); }, {intake}));

  // Intake for a specific duration
  NamedCommands::registerCommand(
      "IntakeFor3Sec",
      frc2::cmd::Sequence(
          frc2::cmd::RunOnce([intake] { intake->Run(); }, {intake}),
          frc2::cmd::Wait(3.0_s),
          frc2::cmd::RunOnce([intake] { intake->Stop(); }, {intake})));

  // ========== SHOOTER COMMANDS ==========
  NamedCommands::registerCommand(
      "ShooterStart",
      frc2::cmd::RunOnce([shooter] { shooter->Shoot(); }, {shooter}));

  NamedCommands::registerCommand(
      "ShooterStop",
      frc2::cmd::RunOnce([shooter] { shooter->Stop(); }, {shooter}));

  NamedCommands::registerCommand(
      "ShootFor2Sec",
      frc2::cmd::Sequence(
          frc2::cmd::RunOnce([shooter] { shooter->Shoot(); }, {shooter}),
          frc2::cmd::Wait(2.0_s),
          frc2::cmd::RunOnce([shooter] { shooter->Stop(); }, {shooter})));

  // ========== TURRET TRACKING COMMANDS ==========
  // Continuous turret tracking - runs until cancelled
  NamedCommands::registerCommand(
      "TurretTrackHub",
      frc2::FunctionalCommand(
          [] {},  // Init
          [turret, camera] {
            int targetTag = GetHubAllianceTag1();
            if (camera->detection.Get() && camera->tagId.Get() == targetTag) {
              turret->PointAtAprilTag(camera->yaw.Get());
            }
          },
          [](bool) {},
          [] { return false; },  // Never ends on its own
          {turret, camera})
          .ToPtr());

  // ========== VISION ALIGNMENT COMMANDS ==========
  // Aligns turret to hub - finishes when yaw < 2 degrees
  NamedCommands::registerCommand(
      "AlignTurretToHub",
      frc2::FunctionalCommand(
          [] {},
          [turret, camera] {
            int targetTag = GetHubAllianceTag1();
            if (camera->detection.Get() && camera->tagId.Get() == targetTag) {
              turret->PointAtAprilTag(camera->yaw.Get());
            }
          },
          [](bool) {},
          [camera] {
            int targetTag = GetHubAllianceTag1();
            return camera->detection.Get() &&
                   camera->tagId.Get() == targetTag &&
                   std::abs(camera->yaw.Get()) < 2.0;
          },
          {turret, camera})
          .ToPtr());

  // Aligns turret to hub tag 2 (for NeutralZone second shot)
  NamedCommands::registerCommand(
      "AlignTurretToHub2",
      frc2::FunctionalCommand(
          [] {},
          [turret, camera] {
            int targetTag = GetHubAllianceTag2();
            if (camera->detection.Get() && camera->tagId.Get() == targetTag) {
              turret->PointAtAprilTag(camera->yaw.Get());
            }
          },
          [](bool) {},
          [camera] {
            int targetTag = GetHubAllianceTag2();
            return camera->detection.Get() &&
                   camera->tagId.Get() == targetTag &&
                   std::abs(camera->yaw.Get()) < 2.0;
          },
          {turret, camera})
          .ToPtr());

  // Aligns turret to ladder - finishes when yaw < 1 degree
  NamedCommands::registerCommand(
      "AlignTurretToLadder",
      frc2::FunctionalCommand(
          [] {},
          [turret, camera] {
            int tag1 = GetLadderTag1();
            int tag2 = GetLadderTag2();
            int currentTag = camera->tagId.Get();
            if (camera->detection.Get() &&
                (currentTag == tag1 || currentTag == tag2)) {
              turret->PointAtAprilTag(camera->yaw.Get());
            }
          },
          [](bool) {},
          [camera] {
            int tag1 = GetLadderTag1();
            int tag2 = GetLadderTag2();
            int currentTag = camera->tagId.Get();
            return camera->detection.Get() &&
                   (currentTag == tag1 || currentTag == tag2) &&
                   std::abs(camera->yaw.Get()) < 1.0;
          },
          {turret, camera})
          .ToPtr());

  // Aligns turret to hub from neutral zone with 20 degree offset
  // Red: 20° right of tag 3, or 20° left of tag 4 (use closest)
  // Blue: 20° left of tag 19, or 20° right of tag 20 (use closest)
  // Finishes when within 5 degrees of target offset
  NamedCommands::registerCommand(
      "AlignTurretToHubNeutral",
      frc2::FunctionalCommand(
          [] {},  // Init
          [turret, camera] {
            int tag1 = GetHubNeutralTag1();  // Tag 3 (red) or 19 (blue)
            int tag2 = GetHubNeutralTag2();  // Tag 4 (red) or 20 (blue)
            int currentTag = camera->tagId.Get();

            if (!camera->detection.Get()) return;
            if (currentTag != tag1 && currentTag != tag2) return;

            double yaw = camera->yaw.Get();
            double offsetYaw;

            // Determine offset based on which tag and alliance
            // Red + tag1(3): 20° right → pass (yaw + 20)
            // Red + tag2(4): 20° left → pass (yaw - 20)
            // Blue + tag1(19): 20° left → pass (yaw - 20)
            // Blue + tag2(20): 20° right → pass (yaw + 20)
            if (currentTag == tag1) {
              // Tag 1: Red goes right, Blue goes left
              offsetYaw = IsRedAlliance() ? (yaw + kNeutralZoneOffset)
                                          : (yaw - kNeutralZoneOffset);
            } else {
              // Tag 2: Red goes left, Blue goes right
              offsetYaw = IsRedAlliance() ? (yaw - kNeutralZoneOffset)
                                          : (yaw + kNeutralZoneOffset);
            }

            turret->PointAtAprilTag(offsetYaw);
          },
          [](bool) {},
          [camera] {
            int tag1 = GetHubNeutralTag1();
            int tag2 = GetHubNeutralTag2();
            int currentTag = camera->tagId.Get();

            if (!camera->detection.Get()) return false;
            if (currentTag != tag1 && currentTag != tag2) return false;

            double yaw = camera->yaw.Get();
            double targetYaw;

            // Target yaw when aligned with offset
            // Red + tag1(3): target yaw = -20 (tag appears 20° left)
            // Red + tag2(4): target yaw = +20 (tag appears 20° right)
            // Blue + tag1(19): target yaw = +20
            // Blue + tag2(20): target yaw = -20
            if (currentTag == tag1) {
              targetYaw = IsRedAlliance() ? -kNeutralZoneOffset : kNeutralZoneOffset;
            } else {
              targetYaw = IsRedAlliance() ? kNeutralZoneOffset : -kNeutralZoneOffset;
            }

            return std::abs(yaw - targetYaw) < 5.0;  // 5 degree tolerance
          },
          {turret, camera})
          .ToPtr());

  // ========== TURRET ABSOLUTE POSITION COMMANDS ==========
  // Turn turret 90 degrees left (counterclockwise) for right ball sweep
  // Finishes when within 5 degrees of target
  NamedCommands::registerCommand(
      "TurretLeft90",
      frc2::FunctionalCommand(
          [] {},  // Init
          [turret] {
            turret->TurnToAngle(90.0);  // +90 = counterclockwise
          },
          [](bool) {},
          [turret] {
            return std::abs(turret->GetAngleDegrees() - 90.0) < 5.0;
          },
          {turret})
          .ToPtr());

  // Turn turret 90 degrees right (clockwise) for left ball sweep
  // Finishes when within 5 degrees of target
  NamedCommands::registerCommand(
      "TurretRight90",
      frc2::FunctionalCommand(
          [] {},  // Init
          [turret] {
            turret->TurnToAngle(-90.0);  // -90 = clockwise
          },
          [](bool) {},
          [turret] {
            return std::abs(turret->GetAngleDegrees() - (-90.0)) < 5.0;
          },
          {turret})
          .ToPtr());

  // Return turret to center (0 degrees / facing backward)
  NamedCommands::registerCommand(
      "TurretCenter",
      frc2::FunctionalCommand(
          [] {},
          [turret] {
            turret->TurnToAngle(0.0);
          },
          [](bool) {},
          [turret] {
            return std::abs(turret->GetAngleDegrees()) < 5.0;
          },
          {turret})
          .ToPtr());

  // ========== CLIMBER COMMANDS ==========
  NamedCommands::registerCommand(
      "ClimbStart",
      frc2::cmd::RunOnce([climber] { climber->Climb(); }, {climber}));

  NamedCommands::registerCommand(
      "ClimbStop",
      frc2::cmd::RunOnce([climber] { climber->Stop(); }, {climber}));

  NamedCommands::registerCommand(
      "ClimbFor1Sec",
      frc2::cmd::Sequence(
          frc2::cmd::RunOnce([climber] { climber->Climb(); }, {climber}),
          frc2::cmd::Wait(2.0_s),
          frc2::cmd::RunOnce([climber] { climber->Stop(); }, {climber})));

  // ========== WAIT COMMANDS ==========
  NamedCommands::registerCommand("WaitForHuman", frc2::cmd::Wait(3.0_s));

  // ========== CLIMB ALIGNMENT COMMAND ==========
  // 3-phase autonomous climb alignment:
  // Phase 1: Vision align to ladder AprilTag
  // Phase 2: Strafe left until collision detected (~38A current spike)
  // Phase 3: Back up until limit switch triggers, then start climb motor
  NamedCommands::registerCommand(
      "ClimbAlignment",
      ClimbAlignmentCommand(drive, camera, climber).ToPtr());
}
