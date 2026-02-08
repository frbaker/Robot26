#include <subsystems/Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>


ShooterSubsystem::ShooterSubsystem(){
   // PID + Feed Forward (new API uses feedForward.V instead of VelocityFF)
   m_leftConfig.closedLoop.P(0.0001).I(0).D(0);
   m_leftConfig.closedLoop.feedForward.kV(0.000176);

   m_rightConfig.closedLoop.P(0.0001).I(0).D(0);
   m_rightConfig.closedLoop.feedForward.kV(0.000176);

   m_feederConfig.closedLoop.P(0.0001).I(0).D(0);
   m_feederConfig.closedLoop.feedForward.kV(0.000176);

   m_collectorConfig.closedLoop.P(0.0001).I(0).D(0);
   m_collectorConfig.closedLoop.feedForward.kV(0.000176);


   m_LeftShooter.Configure(m_leftConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
   m_RightShooter.Configure(m_rightConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
   m_FeederMotor.Configure(m_feederConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
   m_CollectorMotor.Configure(m_collectorConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);

   /*
   Tuning Steps

   1. Start with FF only (P=0, I=0, D=0)
   2. Set FF = 1 / motor_free_speed = 1 / 5676 = 0.000176
   3. Run and check actual RPM vs target on SmartDashboard
   4. Adjust FF until you get close (~95% of target)
   5. Add small P (0.0001) to eliminate remaining error

   If the shooter has a gear ratio (e.g., 2:1 reduction), adjust accordingly:
   Effective Free Speed = 5676 / gear_ratio
   FF = 1 / Effective Free Speed
   For a 2:1 reduction: FF = 1 / 2838 = 0.000352
   */
}

void ShooterSubsystem::Periodic(){
    // See actual vs target RPM
    frc::SmartDashboard::PutNumber("Shooter L RPM", m_LeftEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter R RPM", m_RightEncoder.GetVelocity());
}

void ShooterSubsystem::Shoot(){
    m_LeftController.SetSetpoint(5000, SparkLowLevel::ControlType::kVelocity);
    m_RightController.SetSetpoint(-5000, SparkLowLevel::ControlType::kVelocity);
    m_feederController.SetSetpoint(3500, SparkLowLevel::ControlType::kVelocity);
    m_CollectorController.SetSetpoint(2500, SparkLowLevel::ControlType::kVelocity);
}
//A
void ShooterSubsystem::Stop(){
    m_LeftShooter.Set(0);
    m_RightShooter.Set(0);
    m_FeederMotor.Set(0); //don't forget the feeder?!
    m_CollectorMotor.Set(0);
}

void ShooterSubsystem::ReverseCollector(){
    m_CollectorController.SetSetpoint(-2500, SparkLowLevel::ControlType::kVelocity);
}
