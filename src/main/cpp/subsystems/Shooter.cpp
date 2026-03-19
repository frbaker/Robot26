#include <subsystems/Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <frc2/command/RunCommand.h>


ShooterSubsystem::ShooterSubsystem(){
   // PID + Feed Forward (new API uses feedForward.V instead of VelocityFF)
   // kV = 0.000176 is 1/5676 (NEO free speed) — too low under load
   // kV = 0.000210 compensates for friction/load to get FF closer to target
   /*When you test, watch SmartDashboard. If it's still undershooting, bump kV higher (try 0.000230). If it overshoots or oscillates, back kV down or 
  reduce P. The goal is FF gets you to ~95% of target RPM with P handling the last bit.*/
   m_leftConfig.closedLoop.P(0.0003).I(0).D(0.00001);
   m_leftConfig.closedLoop.feedForward.kV(0.001);
   m_leftConfig.ClosedLoopRampRate(0.001);

   m_rightConfig.closedLoop.P(0.0003).I(0).D(0.00001);
   m_rightConfig.closedLoop.feedForward.kV(0.001);
   m_rightConfig.ClosedLoopRampRate(0.001);

   m_LeftFeederConfig.closedLoop.P(0.0002).I(0).D(0.00001);
   m_LeftFeederConfig.closedLoop.feedForward.kV(0.001);
   m_LeftFeederConfig.ClosedLoopRampRate(0.001);

   m_RightFeederConfig.closedLoop.P(0.0002).I(0).D(0.00001);
   m_RightFeederConfig.closedLoop.feedForward.kV(0.001);
   m_RightFeederConfig.ClosedLoopRampRate(0.001);

   m_collectorConfig.closedLoop.P(0.0001).I(0).D(0);
   m_collectorConfig.closedLoop.feedForward.kV(0.0005);
   m_collectorConfig.ClosedLoopRampRate(0.001);

   m_leftConfig.SmartCurrentLimit(40);
   m_rightConfig.SmartCurrentLimit(40);
   m_LeftFeederConfig.SmartCurrentLimit(40);
   m_RightFeederConfig.SmartCurrentLimit(40);
   m_collectorConfig.SmartCurrentLimit(50);



   m_LeftShooter.Configure(m_leftConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
   m_RightShooter.Configure(m_rightConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
   m_LeftFeederMotor.Configure(m_LeftFeederConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
   m_RightFeederMotor.Configure(m_RightFeederConfig, rev::ResetMode::kResetSafeParameters, rev::PersistMode::kPersistParameters);
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
    frc::SmartDashboard::PutNumber("Shooter L RPM", m_LeftEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Shooter R RPM", m_RightEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Feeder L RPM", m_LeftFeederEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Feeder R RPM", m_RightFeederEncoder.GetVelocity());
    frc::SmartDashboard::PutNumber("Collector RPM", m_CollectorEncoder.GetVelocity());
}

void ShooterSubsystem::Shoot(double rpm){
    if(rpm == 0){
    m_LeftController.SetSetpoint(-ShooterConstants::kShooterRPM, SparkLowLevel::ControlType::kVelocity);
    m_RightController.SetSetpoint(ShooterConstants::kShooterRPM, SparkLowLevel::ControlType::kVelocity);
    }
    else{
        m_LeftController.SetSetpoint(-rpm, SparkLowLevel::ControlType::kVelocity);
        m_RightController.SetSetpoint(rpm, SparkLowLevel::ControlType::kVelocity);
    }
    //m_LeftController.SetSetpoint(4100, SparkLowLevel::ControlType::kVelocity); FOR BASKETBALL
    //m_RightController.SetSetpoint(-4100, SparkLowLevel::ControlType::kVelocity); FOR BASKETBALL
    m_LeftFeederController.SetSetpoint(-3500, SparkLowLevel::ControlType::kVelocity);
    m_RightFeederController.SetSetpoint(3500, SparkLowLevel::ControlType::kVelocity);
   // m_CollectorController.SetSetpoint(2500, SparkLowLevel::ControlType::kVelocity);
}
//A
void ShooterSubsystem::Stop(){
    m_LeftShooter.Set(0);
    m_RightShooter.Set(0);
    m_LeftFeederMotor.Set(0); //don't forget the feeder?!
    m_RightFeederMotor.Set(0);
    m_CollectorMotor.Set(0);
}
void ShooterSubsystem::ReverseCollector(){
    m_CollectorController.SetSetpoint(2500, SparkLowLevel::ControlType::kVelocity);
}

void ShooterSubsystem::ReverseFeeder(){
    m_LeftFeederController.SetSetpoint(3500, SparkLowLevel::ControlType::kVelocity);
    m_RightFeederController.SetSetpoint(-3500, SparkLowLevel::ControlType::kVelocity);
}

void ShooterSubsystem::RunCollector(){
    m_CollectorController.SetSetpoint(-3267, SparkLowLevel::ControlType::kVelocity);
}

void ShooterSubsystem::StopCollector(){
    m_CollectorMotor.Set(0);
}

void ShooterSubsystem::StopFeeder(){
    m_LeftFeederMotor.Set(0);
    m_RightFeederMotor.Set(0);
}


