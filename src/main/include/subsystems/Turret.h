#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>
#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;


class TurretSubsystem : public frc2::SubsystemBase {
    public:
        TurretSubsystem();

        void Periodic() override;

        void PointAtAprilTag(double yaw);

        void SetSpeed(double value);

        void SetPoint(double value);

        double GetPosition();


    private:
        SparkMax m_turretMotor{TurretConstants::kTurretCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_turretEncoder = m_turretMotor.GetEncoder();
        SparkClosedLoopController m_turretController = m_turretMotor.GetClosedLoopController();
        SparkMaxConfig m_turretConfig;
        frc::PIDController anglePIDController{0.00075, 0, 0};

        frc::DigitalInput TurretLimitSwitch{0};
};
