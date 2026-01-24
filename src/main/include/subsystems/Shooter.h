#pragma once
#include <frc2/command/SubsystemBase.h>
#include <frc/controller/PIDController.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <Constants.h>

class ShooterSubsystem : public frc2::SubsystemBase{   
    public:
        ShooterSubsystem();

        void Periodic() override;

        void Shoot();
        void Stop();

    private:
    //A;
        frc::PIDController RightShooterPID{0.4, 0.2, 0};
        frc::PIDController FeederPID{0.4, 0.2, 0};
        frc::PIDController LeftShooterPID{0.4, 0.2, 0};

        rev::spark::SparkMax m_LeftShooter{ShooterConstants::kShooterLeftCanId, rev::spark::SparkLowLevel::MotorType::kBrushless};
        rev::spark::SparkMax m_RightShooter{ShooterConstants::kShooterRightCanId, rev::spark::SparkLowLevel::MotorType::kBrushless};
        rev::spark::SparkRelativeEncoder m_LeftEncoder = m_LeftShooter.GetEncoder();
        rev::spark::SparkRelativeEncoder m_RightEncoder = m_RightShooter.GetEncoder();

        rev::spark::SparkClosedLoopController m_LeftController = m_LeftShooter.GetClosedLoopController();
        rev::spark::SparkClosedLoopController m_RightController = m_RightShooter.GetClosedLoopController();

        rev::spark::SparkMaxConfig m_leftConfig;
        rev::spark::SparkMaxConfig m_rightConfig;
};