#include "subsystems/Shooter.h"

#include <algorithm>
#include <cmath>
#include <frc/smartdashboard/SmartDashboard.h>

ShooterSubsystem::ShooterSubsystem() {
  m_leftMotor.Configure(
      Configs::Shooter::LeftConfig(),
      rev::spark::SparkBase::ResetMode::kResetSafeParameters,
      rev::spark::SparkBase::PersistMode::kPersistParameters);

  m_rightMotor.Configure(
      Configs::Shooter::RightConfig(),
      rev::spark::SparkBase::ResetMode::kResetSafeParameters,
      rev::spark::SparkBase::PersistMode::kPersistParameters);
}

void ShooterSubsystem::Periodic() {
  frc::SmartDashboard::PutNumber("Shooter/Left RPM", GetLeftVelocity());
  frc::SmartDashboard::PutNumber("Shooter/Right RPM", GetRightVelocity());
  frc::SmartDashboard::PutNumber("Shooter/Target RPM", m_targetRPM);
  frc::SmartDashboard::PutBoolean("Shooter/At Speed", AtTargetVelocity());
}

void ShooterSubsystem::Shoot() {
  SetVelocity(ShooterConstants::kDefaultTargetRPM);
}

void ShooterSubsystem::SetVelocity(double rpm) {
  m_targetRPM = std::clamp(rpm, 0.0, ShooterConstants::kMaxRPM);

  m_leftController.SetReference(
      m_targetRPM,
      rev::spark::SparkLowLevel::ControlType::kVelocity);

  m_rightController.SetReference(
      m_targetRPM,
      rev::spark::SparkLowLevel::ControlType::kVelocity);
}

void ShooterSubsystem::Stop() {
  m_targetRPM = 0.0;
  m_leftMotor.Set(0.0);
  m_rightMotor.Set(0.0);
}

bool ShooterSubsystem::AtTargetVelocity() const {
  if (m_targetRPM == 0.0) {
    return false;
  }

  double leftError = std::abs(GetLeftVelocity() - m_targetRPM);
  double rightError = std::abs(GetRightVelocity() - m_targetRPM);

  return leftError < ShooterConstants::kVelocityToleranceRPM &&
         rightError < ShooterConstants::kVelocityToleranceRPM;
}

double ShooterSubsystem::GetLeftVelocity() const {
  return m_leftEncoder.GetVelocity();
}

double ShooterSubsystem::GetRightVelocity() const {
  return m_rightEncoder.GetVelocity();
}
