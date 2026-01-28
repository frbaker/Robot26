// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

#include "subsystems/DriveSubsystem.h"

#include <frc/geometry/Rotation2d.h>
#include <hal/FRCUsageReporting.h>
#include <units/angle.h>
#include <units/angular_velocity.h>
#include <units/velocity.h>

#include "Constants.h"

using namespace DriveConstants;

DriveSubsystem::DriveSubsystem()
    : m_gyro(kPigeonCanId),
      m_frontLeft{kFrontLeftDrivingCanId, kFrontLeftTurningCanId,
                  kFrontLeftChassisAngularOffset},
      m_rearLeft{kRearLeftDrivingCanId, kRearLeftTurningCanId,
                 kRearLeftChassisAngularOffset},
      m_frontRight{kFrontRightDrivingCanId, kFrontRightTurningCanId,
                   kFrontRightChassisAngularOffset},
      m_rearRight{kRearRightDrivingCanId, kRearRightTurningCanId,
                  kRearRightChassisAngularOffset},
      m_odometry{kDriveKinematics,
                 frc::Rotation2d(units::radian_t{m_gyro.GetRotation2d().Radians()}),
                 {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
                  m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
                 frc::Pose2d{}} {
  // Usage reporting for MAXSwerve template
  HAL_Report(HALUsageReporting::kResourceType_RobotDrive,
             HALUsageReporting::kRobotDriveSwerve_MaxSwerve);
}

void DriveSubsystem::Periodic() {
  // Implementation of subsystem periodic method goes here.

  // ==========================================================================
  // BUG: MODULE STATE ORDER MISMATCH
  // ==========================================================================
  // The order of modules passed to m_odometry.Update() MUST match the order
  // used when constructing the SwerveDriveKinematics and SwerveDriveOdometry.
  //
  // In the constructor (lines 27-31), the odometry was initialized with:
  //   {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
  //    m_rearLeft.GetPosition(), m_rearRight.GetPosition()}
  //
  // This establishes the expected order as:
  //   [0] = Front Left
  //   [1] = Front Right
  //   [2] = Rear Left
  //   [3] = Rear Right
  //
  // However, the current code below uses a DIFFERENT order:
  //   [0] = Front Left   (correct)
  //   [1] = Rear Left    (WRONG - should be Front Right)
  //   [2] = Front Right  (WRONG - should be Rear Left)
  //   [3] = Rear Right   (correct)
  //
  // This causes the odometry to incorrectly interpret wheel positions,
  // resulting in inaccurate pose estimation. The robot will think it's
  // moving/rotating differently than it actually is.
  //
  // FIX: Change the order below to match the constructor:
  //   {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
  //    m_rearLeft.GetPosition(), m_rearRight.GetPosition()}
  // ==========================================================================

  m_odometry.Update(frc::Rotation2d(units::radian_t{m_gyro.GetRotation2d().Radians()}),
                    {m_frontLeft.GetPosition(), m_rearLeft.GetPosition(),
                     m_frontRight.GetPosition(), m_rearRight.GetPosition()});
}

void DriveSubsystem::Drive(units::meters_per_second_t xSpeed,
                           units::meters_per_second_t ySpeed,
                           units::radians_per_second_t rot,
                           bool fieldRelative) {
  // Convert the commanded speeds into the correct units for the drivetrain
  units::meters_per_second_t xSpeedDelivered =
      xSpeed.value() * DriveConstants::kMaxSpeed;
  units::meters_per_second_t ySpeedDelivered =
      ySpeed.value() * DriveConstants::kMaxSpeed;
  units::radians_per_second_t rotDelivered =
      rot.value() * DriveConstants::kMaxAngularSpeed;

  auto states = kDriveKinematics.ToSwerveModuleStates(
      fieldRelative
          ? frc::ChassisSpeeds::FromFieldRelativeSpeeds(
                xSpeedDelivered, ySpeedDelivered, rotDelivered,
                frc::Rotation2d(units::radian_t{m_gyro.GetRotation2d().Radians()}))
          : frc::ChassisSpeeds{xSpeedDelivered, ySpeedDelivered, rotDelivered});

  kDriveKinematics.DesaturateWheelSpeeds(&states, DriveConstants::kMaxSpeed);

  auto [fl, fr, bl, br] = states;

  m_frontLeft.SetDesiredState(fl);
  m_frontRight.SetDesiredState(fr);
  m_rearLeft.SetDesiredState(bl);
  m_rearRight.SetDesiredState(br);
}

void DriveSubsystem::SetX() {
  m_frontLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
  m_frontRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearLeft.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{-45_deg}});
  m_rearRight.SetDesiredState(
      frc::SwerveModuleState{0_mps, frc::Rotation2d{45_deg}});
}

void DriveSubsystem::SetModuleStates(
    wpi::array<frc::SwerveModuleState, 4> desiredStates) {
  kDriveKinematics.DesaturateWheelSpeeds(&desiredStates,
                                         DriveConstants::kMaxSpeed);
  m_frontLeft.SetDesiredState(desiredStates[0]);
  m_frontRight.SetDesiredState(desiredStates[1]);
  m_rearLeft.SetDesiredState(desiredStates[2]);
  m_rearRight.SetDesiredState(desiredStates[3]);
}

void DriveSubsystem::ResetEncoders() {
  m_frontLeft.ResetEncoders();
  m_rearLeft.ResetEncoders();
  m_frontRight.ResetEncoders();
  m_rearRight.ResetEncoders();
}

units::degree_t DriveSubsystem::GetHeading() const {
  return frc::Rotation2d(
             units::radian_t{m_gyro.GetRotation2d().Radians()})
      .Degrees();
}

void DriveSubsystem::ZeroHeading() { m_gyro.Reset(); }

double DriveSubsystem::GetTurnRate() {
  return -m_gyro.GetAngularVelocityZWorld().GetValueAsDouble();
}

frc::Pose2d DriveSubsystem::GetPose() { return m_odometry.GetPose(); }

void DriveSubsystem::ResetOdometry(frc::Pose2d pose) {
  m_odometry.ResetPosition(
      GetHeading(),
      {m_frontLeft.GetPosition(), m_frontRight.GetPosition(),
       m_rearLeft.GetPosition(), m_rearRight.GetPosition()},
      pose);
}
