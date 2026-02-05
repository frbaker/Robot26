#include <subsystems/Turret.h>
#include <algorithm>

TurretSubsystem::TurretSubsystem() {
    // Reset encoder on startup (assumes turret starts at home/zero position)
    m_encoder.SetPosition(0.0);
}

void TurretSubsystem::Periodic() {
}

void TurretSubsystem::PointAtAprilTag(double yaw) {
    double rotation = anglePIDController.Calculate(yaw, 0.0);
    m_turretMotor.Set(rotation / 75);
}

void TurretSubsystem::TurnToAngle(double angleDegrees) {
    double currentAngle = GetAngleDegrees();
    double output = positionPIDController.Calculate(currentAngle, angleDegrees);
    // Clamp output to reasonable motor speed
    output = std::clamp(output, -0.5, 0.5);
    m_turretMotor.Set(output);
}

double TurretSubsystem::GetAngleDegrees() {
    double rotations = m_encoder.GetPosition();
    return rotations / TurretConstants::kRotationsPerDegree;
}

void TurretSubsystem::ResetEncoder() {
    m_encoder.SetPosition(0.0);
}