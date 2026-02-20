#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>
#include <frc/DigitalInput.h>

using namespace rev::spark;


class TurretSubsystem : public frc2::SubsystemBase {
    public:
        TurretSubsystem();

        void Periodic() override;

        void PointAtAprilTag(double yaw);

        void SetSpeed(double value);


    private:
        SparkMax m_turretMotor{TurretConstants::kTurretCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_turretEncoder = m_turretMotor.GetEncoder();
        frc::PIDController anglePIDController{0.0005, 0, 0};

        frc::DigitalInput TurretLimitSwitch{0};
};
