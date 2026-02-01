#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <Constants.h>

using namespace rev::spark;

class ShooterSubsystem : public frc2::SubsystemBase{   
    public:
        ShooterSubsystem();

        void Periodic() override;

        void Shoot();
        void Stop();

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
        SparkMax m_FeederMotor{ShooterConstants::kFeederCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_feederEncoder = m_FeederMotor.GetEncoder();
        SparkClosedLoopController m_feederController = m_FeederMotor.GetClosedLoopController();
        SparkMaxConfig m_feederConfig;
};