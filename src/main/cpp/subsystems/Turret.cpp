#include <subsystems/Turret.h>



TurretSubsystem::TurretSubsystem(){

}


void TurretSubsystem::Periodic(){

}

void TurretSubsystem::PointAtAprilTag(double yaw){
    double rotation = anglePIDController.Calculate(yaw, 0.0);
    //units::radians_per_second_t rotationsPerSecond{rotation/75};
    m_turretMotor.Set(rotation);
    //Here is you problem with the turrent tracking april tags. 
    // For example: if the camera reports a yaw of 10 degrees, the motor gets: 0.025 * 10 / 75 = 0.0033 or (0.33% motor power) which is not enough to make it move beyond maybe the initial twitch
    //I recommend getting rid of the division by 75 and just using the P of the PID loop at first.
}

void TurretSubsystem::SetSpeed(double value){
    m_turretMotor.Set(value*0.2);
}