#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>

#include "Constants.h"
#include "Configs.h"

class ShooterSubsystem : public frc2::SubsystemBase {
 public:
  ShooterSubsystem();

  void Periodic() override;

  /**
   * Run the shooter at the default target RPM.
   */
  void Shoot();

  /**
   * Run the shooter at a specific RPM.
   * @param rpm Target velocity in RPM (clamped to 0-5000)
   */
  void SetVelocity(double rpm);

  /**
   * Stop the shooter motors.
   */
  void Stop();

  /**
   * Check if the shooter is at the target velocity.
   * @return true if within tolerance of target RPM
   */
  bool AtTargetVelocity() const;

  /**
   * Get the current velocity of the left shooter motor.
   * @return Velocity in RPM
   */
  double GetLeftVelocity() const;

  /**
   * Get the current velocity of the right shooter motor.
   * @return Velocity in RPM
   */
  double GetRightVelocity() const;

 private:
  rev::spark::SparkMax m_leftMotor{
      ShooterConstants::kShooterLeftCanId,
      rev::spark::SparkLowLevel::MotorType::kBrushless};
  rev::spark::SparkMax m_rightMotor{
      ShooterConstants::kShooterRightCanId,
      rev::spark::SparkLowLevel::MotorType::kBrushless};

  rev::spark::SparkRelativeEncoder m_leftEncoder = m_leftMotor.GetEncoder();
  rev::spark::SparkRelativeEncoder m_rightEncoder = m_rightMotor.GetEncoder();

  rev::spark::SparkClosedLoopController m_leftController =
      m_leftMotor.GetClosedLoopController();
  rev::spark::SparkClosedLoopController m_rightController =
      m_rightMotor.GetClosedLoopController();

  double m_targetRPM = 0.0;
};
