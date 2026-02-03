#include <subsystems/Shooter.h>
#include <frc/smartdashboard/SmartDashboard.h>
#include <cmath>


ShooterSubsystem::ShooterSubsystem(){
   // PID + Feed Forward
   // kV = 1 / free_speed = 1 / 5676 = 0.000176
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
    m_LeftController.SetSetpoint(-1600, SparkLowLevel::ControlType::kVelocity);
    m_RightController.SetSetpoint(-1600, SparkLowLevel::ControlType::kVelocity);
    m_feederController.SetSetpoint(-1200, SparkLowLevel::ControlType::kVelocity);
    m_collectorController.SetSetpoint(-1200, SparkLowLevel::ControlType::kVelocity);
}

void ShooterSubsystem::Stop(){
    m_LeftShooter.Set(0);
    m_RightShooter.Set(0);
    m_FeederMotor.Set(0);
    m_CollectorMotor.Set(0);
}
double ShooterSubsystem::CalculateRPMFromDistance(double distanceFeet) {
    // Lookup table - UPDATE THESE VALUES after testing!
    // =================================================
    // Distance in FEET (e.g., 1'6" = 1.5 feet)
    constexpr int TABLE_SIZE = 5;
    constexpr double distances[TABLE_SIZE] = {1.0, 1.5, 2.0, 2.5, 3.0};  // feet
    constexpr double rpms[TABLE_SIZE]      = {1200, 1400, 1600, 1900, 2200};

    // Handle out-of-range: clamp to table bounds
    if (distanceFeet <= distances[0]) {
        return rpms[0];
    }
    if (distanceFeet >= distances[TABLE_SIZE - 1]) {
        return rpms[TABLE_SIZE - 1];
    }

    // Find the two data points we're between and interpolate
    for (int i = 0; i < TABLE_SIZE - 1; i++) {
        if (distanceFeet >= distances[i] && distanceFeet < distances[i + 1]) {
            // Linear interpolation: rpm = rpm1 + (rpm2-rpm1) * (dist-dist1)/(dist2-dist1)
            double t = (distanceFeet - distances[i]) / (distances[i + 1] - distances[i]);
            return rpms[i] + t * (rpms[i + 1] - rpms[i]);
        }
    }

    return 1600;  // Fallback
}

void ShooterSubsystem::ShootAtDistance(double distanceFeet) {
    double targetRPM = CalculateRPMFromDistance(distanceFeet);

    // Safety limits
    constexpr double MIN_RPM = 1000;
    constexpr double MAX_RPM = 3000;
    constexpr double RPM_TOLERANCE = 100;  // How close to target speed before feeding

    if (targetRPM < MIN_RPM) targetRPM = MIN_RPM;
    if (targetRPM > MAX_RPM) targetRPM = MAX_RPM;

    // Get actual shooter speeds (they're negative, so use abs)
    double leftRPM = std::abs(m_LeftEncoder.GetVelocity());
    double rightRPM = std::abs(m_RightEncoder.GetVelocity());
    bool atSpeed = (leftRPM >= targetRPM - RPM_TOLERANCE) &&
                   (rightRPM >= targetRPM - RPM_TOLERANCE);

    // Display on SmartDashboard for debugging
    frc::SmartDashboard::PutNumber("Target Distance (ft)", distanceFeet);
    frc::SmartDashboard::PutNumber("Target RPM", targetRPM);
    frc::SmartDashboard::PutBoolean("Shooter At Speed", atSpeed);

    // Set shooter motors (negative because they spin backward)
    m_LeftController.SetSetpoint(-targetRPM, SparkLowLevel::ControlType::kVelocity);
    m_RightController.SetSetpoint(-targetRPM, SparkLowLevel::ControlType::kVelocity);

    // Only run feeder and collector once shooter is up to speed
    if (atSpeed) {
        m_feederController.SetSetpoint(-1200, SparkLowLevel::ControlType::kVelocity);
        m_collectorController.SetSetpoint(-1200, SparkLowLevel::ControlType::kVelocity);
    } else {
        m_FeederMotor.Set(0);
        m_CollectorMotor.Set(0);
    }
}
