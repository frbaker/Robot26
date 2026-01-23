#include <subsystems/Shooter.h>

ShooterSubsystem :: ShooterSubsystem(){
   
}
void ShooterSubsystem::Periodic(){
    
}

void ShooterSubsystem::Shoot(){ //for short people ONLY (if you arent short then its ok though)
    m_LeftShooter.Set(LeftShooterPID.Calculate(m_LeftEncoder.GetVelocity()));
    m_RightShooter.Set(RightShooterPID.Calculate(m_RightEncoder.GetVelocity()));
}

void ShooterSubsystem::Stop(){
    m_LeftShooter.Set(0);
    m_RightShooter.Set(0);
}
