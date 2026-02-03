#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <Constants.h>
#include <rev/config/FeedForwardConfig.h>

using namespace rev::spark;

class ShooterSubsystem : public frc2::SubsystemBase{
    public:
        ShooterSubsystem();

        void Periodic() override;

        void Shoot();
        void Stop();

        void ShootAtDistance(double distanceFeet);

    private:
        double CalculateRPMFromDistance(double distanceFeet);

        SparkMax m_LeftShooter{ShooterConstants::kShooterLeftCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_LeftEncoder = m_LeftShooter.GetEncoder();
        SparkClosedLoopController m_LeftController = m_LeftShooter.GetClosedLoopController();
        SparkMaxConfig m_leftConfig;

        SparkMax m_RightShooter{ShooterConstants::kShooterRightCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_RightEncoder = m_RightShooter.GetEncoder();
        SparkClosedLoopController m_RightController = m_RightShooter.GetClosedLoopController();
        SparkMaxConfig m_rightConfig;

        SparkMax m_FeederMotor{ShooterConstants::kFeederCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_feederEncoder = m_FeederMotor.GetEncoder();
        SparkClosedLoopController m_feederController = m_FeederMotor.GetClosedLoopController();
        SparkMaxConfig m_feederConfig;

        SparkMax m_CollectorMotor{ShooterConstants::kCollectorCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_collectorEncoder = m_CollectorMotor.GetEncoder();
        SparkClosedLoopController m_collectorController = m_CollectorMotor.GetClosedLoopController();
        SparkMaxConfig m_collectorConfig;
};
