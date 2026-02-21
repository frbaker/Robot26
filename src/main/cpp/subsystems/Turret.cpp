#include <subsystems/Turret.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <algorithm>

TurretSubsystem::TurretSubsystem(){
    m_turretConfig.softLimit.ForwardSoftLimit(TurretConstants::kTurretMaximum);
    m_turretConfig.softLimit.ReverseSoftLimit(TurretConstants::kTurretMinimum);
}

void TurretSubsystem::Periodic(){
    bool LimitSwitchTriggered = !TurretLimitSwitch.Get();
    frc::SmartDashboard::PutBoolean("turret limit switch", LimitSwitchTriggered);
    frc::SmartDashboard::PutNumber("turret value", m_turretEncoder.GetPosition());
    if(LimitSwitchTriggered){
        m_turretEncoder.SetPosition(0);
    }
}

void TurretSubsystem::PointAtAprilTag(double yaw) {
    double rotation = anglePIDController.Calculate(yaw, 0.0);
    m_turretMotor.Set(rotation);
    if(rotation > 0){
        if(m_turretEncoder.GetPosition() > TurretConstants::kTurretMaximum){
            m_turretMotor.Set(0);
        }
    }
    if(rotation < 0){
        if(m_turretEncoder.GetPosition() < TurretConstants::kTurretMinimum){
            m_turretMotor.Set(0);
        }
    }
}

void TurretSubsystem::SetSpeed(double value){
    m_turretMotor.Set(value);
    if(value > 0){
        if(m_turretEncoder.GetPosition() > TurretConstants::kTurretMaximum){
            m_turretMotor.Set(0);
        }
    }
    if(value < 0){
        if(m_turretEncoder.GetPosition() < TurretConstants::kTurretMinimum){
            m_turretMotor.Set(0);
        }
    }
}

void TurretSubsystem::TurnToAngle(double angleDegrees) {
    double currentAngle = GetAngleDegrees();
    double output = positionPIDController.Calculate(currentAngle, angleDegrees);
    // Clamp output to reasonable motor speed
    output = std::clamp(output, -0.5, 0.5);
    m_turretMotor.Set(output);
}

double TurretSubsystem::GetAngleDegrees() {
    double rotations = m_turretEncoder.GetPosition();
    return rotations / TurretConstants::kRotationsPerDegree;
}

void TurretSubsystem::ResetEncoder() {
    m_turretEncoder.SetPosition(0.0);
}
