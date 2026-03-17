#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <rev/SparkFlex.h>
#include <rev/config/SparkFlexConfig.h>
#include <Constants.h>
#include <rev/config/FeedForwardConfig.h>
#include <frc/motorcontrol/PWMSparkMax.h>

using namespace rev::spark;

class ShooterSubsystem : public frc2::SubsystemBase{   
    public:
        ShooterSubsystem();

        void Periodic() override;

        void Shoot(double rpm = 0);
        void Stop();

        void ReverseCollector();
        void ReverseFeeder();
        void RunCollector();

        void RunSpindexer();
        void StopSpindexer();
        void ReverseSpindexer();

        void StopCollector();
        void StopFeeder();
        

    private:
    //A;

        SparkMax m_LeftShooter{ShooterConstants::kShooterLeftCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_LeftEncoder = m_LeftShooter.GetEncoder();
        SparkClosedLoopController m_LeftController = m_LeftShooter.GetClosedLoopController();
        SparkMaxConfig m_leftConfig;

        SparkMax m_RightShooter{ShooterConstants::kShooterRightCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_RightEncoder = m_RightShooter.GetEncoder();
        SparkClosedLoopController m_RightController = m_RightShooter.GetClosedLoopController();
        SparkMaxConfig m_rightConfig;

//A A A A A A A A A A A A A A A A A A A A A A A A A?!?!?!?!?!?!?!?!
        SparkFlex m_LeftFeederMotor{ShooterConstants::kLeftFeederCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_LeftFeederEncoder = m_LeftFeederMotor.GetEncoder();
        SparkClosedLoopController m_LeftFeederController = m_LeftFeederMotor.GetClosedLoopController();
        SparkFlexConfig m_LeftFeederConfig;

        SparkFlex m_RightFeederMotor{ShooterConstants::kRightFeederCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_RightFeederEncoder = m_RightFeederMotor.GetEncoder();
        SparkClosedLoopController m_RightFeederController = m_RightFeederMotor.GetClosedLoopController();
        SparkFlexConfig m_RightFeederConfig;

        SparkFlex m_CollectorMotor{ShooterConstants::kCollectorCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_CollectorEncoder = m_CollectorMotor.GetEncoder();
        SparkClosedLoopController m_CollectorController = m_CollectorMotor.GetClosedLoopController();
        SparkFlexConfig m_collectorConfig;
};