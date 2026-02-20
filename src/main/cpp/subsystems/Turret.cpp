#include <subsystems/Turret.h>
#include <frc/smartdashboard/SmartDashboard.h>



TurretSubsystem::TurretSubsystem(){

}


void TurretSubsystem::Periodic(){
    bool LimitSwitchTriggered = TurretLimitSwitch.Get();
    frc::SmartDashboard::PutBoolean("turret limit switch", LimitSwitchTriggered);
}

void TurretSubsystem::PointAtAprilTag(double yaw){
    double rotation = anglePIDController.Calculate(yaw, 0.0);
    //units::radians_per_second_t rotationsPerSecond{rotation/75};
    m_turretMotor.Set(rotation);
    //0.025 * 10 / 2 = 0.125
    //Maybe /3 because 0.1 might be a little fast
}

void TurretSubsystem::SetSpeed(double value){
    m_turretMotor.Set(value);
}