#include <subsystems/Turret.h>
#include <frc/smartdashboard/SmartDashboard.h>




TurretSubsystem::TurretSubsystem(){
    m_turretConfig.softLimit.ForwardSoftLimit(TurretConstants::kTurretMaximum);
    m_turretConfig.softLimit.ReverseSoftLimit(TurretConstants::kTurretMinimum);

    m_turretMotor.Configure(m_turretConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
}


void TurretSubsystem::Periodic(){
    bool LimitSwitchTriggered = !TurretLimitSwitch.Get();
    frc::SmartDashboard::PutBoolean("turret limit switch", LimitSwitchTriggered);
    frc::SmartDashboard::PutNumber("turret value", m_turretEncoder.GetPosition());
    if(LimitSwitchTriggered){
        m_turretEncoder.SetPosition(0);
    }
}

void TurretSubsystem::PointAtAprilTag(double yaw){
    double rotation = anglePIDController.Calculate(yaw, 0.0);
    //units::radians_per_second_t rotationsPerSecond{rotation/75};
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
    //0.025 * 10 / 2 = 0.125
    //Maybe /3 because 0.1 might be a little fast
}

double TurretSubsystem::GetPosition() {
    return m_turretEncoder.GetPosition();
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