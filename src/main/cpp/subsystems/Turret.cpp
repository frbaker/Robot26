#include <subsystems/Turret.h>



TurretSubsystem::TurretSubsystem(){

}


void TurretSubsystem::Periodic(){

}

void TurretSubsystem::PointAtAprilTag(double yaw){
    double rotation = anglePIDController.Calculate(yaw, 0.0);
    //units::radians_per_second_t rotationsPerSecond{rotation/75};
    m_turretMotor.Set(rotation/75);
}