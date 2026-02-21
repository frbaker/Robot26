// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "commands/ClimbAlignmentCommand.h"
#include <frc/DriverStation.h>
#include <cmath>

using namespace ClimbAlignmentConstants;

ClimbAlignmentCommand::ClimbAlignmentCommand(DriveSubsystem* drive,
                                             CameraSubsystem* camera,
                                             ClimberSubsystem* climber)
    : m_drive(drive), m_camera(camera), m_climber(climber) {
  AddRequirements({drive, camera, climber});
}

void ClimbAlignmentCommand::Initialize() {
  m_currentPhase = Phase::kVisionAlign;
  m_phaseTimer.Reset();
  m_phaseTimer.Start();
  m_limitSwitchDebounceCount = 0;
  m_collisionDetector.Reset();

  frc::SmartDashboard::PutString("ClimbAlign/Phase", "Phase1_VisionAlign");
}

void ClimbAlignmentCommand::Execute() {
  LogToDashboard();

  switch (m_currentPhase) {
    case Phase::kVisionAlign:
      ExecutePhase1_VisionAlign();
      break;
    case Phase::kStrafeIntoUpright:
      ExecutePhase2_StrafeIntoUpright();
      break;
    case Phase::kReverseToLimitSwitch:
      ExecutePhase3_ReverseToLimitSwitch();
      break;
    case Phase::kDone:
      // Nothing to do, IsFinished will return true
      break;
  }
}

void ClimbAlignmentCommand::ExecutePhase1_VisionAlign() {
  int targetTag = GetLadderTag();

  // Check for timeout
  if (m_phaseTimer.Get().value() > kPhase1TimeoutSeconds) {
    // Timeout - abort or move to next phase depending on requirements
    m_currentPhase = Phase::kDone;
    frc::SmartDashboard::PutString("ClimbAlign/Phase", "TIMEOUT_Phase1");
    return;
  }

  // Check if we can see the tag
  if (!m_camera->GetDetection() || m_camera->GetTagId() != targetTag) {
    // Can't see tag - stop and wait
    m_drive->Drive(0_mps, 0_mps, 0_rad_per_s, false);
    return;
  }

  // Get vision data
  double yaw = m_camera->GetYaw();           // Degrees, positive = tag to the right
  double distance = m_camera->GetDistance(); // Distance to tag (feet from camera)

  // Calculate errors
  // Target distance: wall distance minus upright offset minus camera-to-bumper offset plus standoff
  double targetDistance = kUprightToWallMeters - kCameraToFrontBumperMeters +
                          kDesiredBumperStandoffMeters;
  double distanceError = distance - targetDistance;

  // Lateral error: we want to be kRightOffsetMeters to the right of the tag centerline
  // If yaw is positive, tag is to our right, so we're left of it
  // Convert yaw to approximate lateral position (rough approximation using distance)
  double lateralPosition = distance * std::tan(yaw * M_PI / 180.0);
  double lateralError = lateralPosition - kRightOffsetMeters;

  // Yaw error: we want yaw = 0 (square to wall)
  double yawError = yaw;

  // Check if within tolerance
  bool distanceOk = std::abs(distanceError) < kDistanceToleranceMeters;
  bool lateralOk = std::abs(lateralError) < kLateralToleranceMeters;
  bool yawOk = std::abs(yawError) < kYawToleranceDegrees;

  if (distanceOk && lateralOk && yawOk) {
    // Aligned! Move to phase 2
    m_currentPhase = Phase::kStrafeIntoUpright;
    m_phaseTimer.Reset();
    m_headingSnapshot = m_drive->GetHeading().value();
    m_collisionDetector.Reset();
    frc::SmartDashboard::PutString("ClimbAlign/Phase", "Phase2_StrafeIntoUpright");
    return;
  }

  // Calculate drive outputs using proportional control
  double xSpeed = -distanceError * kDistanceP;    // Forward/backward
  double ySpeed = -lateralError * kLateralP;      // Strafe left/right
  double rotSpeed = -yawError * kYawP;            // Rotation

  // Clamp speeds
  xSpeed = std::clamp(xSpeed, -1.0, 1.0);
  ySpeed = std::clamp(ySpeed, -1.0, 1.0);
  rotSpeed = std::clamp(rotSpeed, -1.0, 1.0);

  // Drive robot-relative
  m_drive->Drive(units::meters_per_second_t{xSpeed},
                 units::meters_per_second_t{ySpeed},
                 units::radians_per_second_t{rotSpeed}, false);
}

void ClimbAlignmentCommand::ExecutePhase2_StrafeIntoUpright() {
  // Check for timeout
  if (m_phaseTimer.Get().value() > kPhase2TimeoutSeconds) {
    m_currentPhase = Phase::kDone;
    frc::SmartDashboard::PutString("ClimbAlign/Phase", "TIMEOUT_Phase2");
    return;
  }

  // Heading lock - keep robot square
  double currentHeading = m_drive->GetHeading().value();
  double headingError = currentHeading - m_headingSnapshot;
  // Normalize heading error to -180 to 180
  while (headingError > 180) headingError -= 360;
  while (headingError < -180) headingError += 360;
  double rotCorrection = -headingError * kHeadingLockP;

  // Get actual velocity for collision detection
  auto speeds = m_drive->GetRobotRelativeSpeeds();
  double actualLateralVelocity = speeds.vy.value();
  double commandedLateralVelocity = kStrafeSpeedMps;

  // Get average drive current
  double avgCurrent = m_drive->GetAverageDriveCurrent();

  // Update collision detector
  bool collision = m_collisionDetector.Update(avgCurrent, commandedLateralVelocity,
                                               std::abs(actualLateralVelocity));

  if (collision) {
    // Collision detected! Move to phase 3
    m_currentPhase = Phase::kReverseToLimitSwitch;
    m_phaseTimer.Reset();
    m_limitSwitchDebounceCount = 0;
    frc::SmartDashboard::PutString("ClimbAlign/Phase", "Phase3_ReverseToLimitSwitch");
    return;
  }

  // Strafe left (positive Y in robot-relative frame) with heading lock
  m_drive->Drive(0_mps, units::meters_per_second_t{kStrafeSpeedMps},
                 units::radians_per_second_t{rotCorrection}, false);
}

void ClimbAlignmentCommand::ExecutePhase3_ReverseToLimitSwitch() {
  // Check for timeout
  if (m_phaseTimer.Get().value() > kPhase3TimeoutSeconds) {
    m_currentPhase = Phase::kDone;
    frc::SmartDashboard::PutString("ClimbAlign/Phase", "TIMEOUT_Phase3");
    return;
  }

  // Heading lock
  double currentHeading = m_drive->GetHeading().value();
  double headingError = currentHeading - m_headingSnapshot;
  while (headingError > 180) headingError -= 360;
  while (headingError < -180) headingError += 360;
  double rotCorrection = -headingError * kHeadingLockP;

  // Check limit switch with debounce
  if (m_climber->IsLimitSwitchTriggered()) {
    m_limitSwitchDebounceCount++;
    if (m_limitSwitchDebounceCount >= kLimitSwitchDebounceCount) {
      // Limit switch confirmed!
      m_currentPhase = Phase::kDone;
      frc::SmartDashboard::PutString("ClimbAlign/Phase", "COMPLETE");
      return;
    }
  } else {
    m_limitSwitchDebounceCount = 0;
  }

  // Drive backward (negative X) with heading lock
  m_drive->Drive(units::meters_per_second_t{-kReverseSpeedMps}, 0_mps,
                 units::radians_per_second_t{rotCorrection}, false);
}

void ClimbAlignmentCommand::End(bool interrupted) {
  // Stop driving
  m_drive->Drive(0_mps, 0_mps, 0_rad_per_s, false);
  m_phaseTimer.Stop();

  if (!interrupted && m_currentPhase == Phase::kDone) {
    // Successfully aligned - start climbing
    m_climber->Climb();
    frc::SmartDashboard::PutBoolean("ClimbAlign/ClimbStarted", true);
  } else {
    frc::SmartDashboard::PutBoolean("ClimbAlign/ClimbStarted", false);
  }
}

bool ClimbAlignmentCommand::IsFinished() {
  return m_currentPhase == Phase::kDone;
}

int ClimbAlignmentCommand::GetLadderTag() {
  auto alliance = frc::DriverStation::GetAlliance();
  if (alliance.has_value() &&
      alliance.value() == frc::DriverStation::Alliance::kRed) {
    return AprilTagConstants::Red::kLadder1;
  }
  return AprilTagConstants::Blue::kLadder1;
}

void ClimbAlignmentCommand::LogToDashboard() {
  frc::SmartDashboard::PutNumber("ClimbAlign/AvgDriveCurrent",
                                  m_drive->GetAverageDriveCurrent());

  auto speeds = m_drive->GetRobotRelativeSpeeds();
  frc::SmartDashboard::PutNumber("ClimbAlign/VelX", speeds.vx.value());
  frc::SmartDashboard::PutNumber("ClimbAlign/VelY", speeds.vy.value());

  frc::SmartDashboard::PutNumber("ClimbAlign/Heading", m_drive->GetHeading().value());

  if (m_currentPhase == Phase::kStrafeIntoUpright) {
    frc::SmartDashboard::PutNumber("ClimbAlign/HeadingError",
                                    m_drive->GetHeading().value() - m_headingSnapshot);
    frc::SmartDashboard::PutBoolean("ClimbAlign/CollisionDetected",
                                     m_collisionDetector.IsCollisionDetected());
    frc::SmartDashboard::PutNumber("ClimbAlign/CollisionStateTime",
                                    m_collisionDetector.GetCollisionStateTime());
  }

  frc::SmartDashboard::PutBoolean("ClimbAlign/LimitSwitch",
                                   m_climber->IsLimitSwitchTriggered());
  frc::SmartDashboard::PutNumber("ClimbAlign/LimitSwitchDebounce",
                                  m_limitSwitchDebounceCount);
  frc::SmartDashboard::PutNumber("ClimbAlign/PhaseTime", m_phaseTimer.Get().value());
}
