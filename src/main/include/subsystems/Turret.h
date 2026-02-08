#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <rev/SparkMax.h>
#include <frc/controller/PIDController.h>

using namespace rev::spark;


class TurretSubsystem : public frc2::SubsystemBase {
    public:
        TurretSubsystem();

        void Periodic() override;

        void PointAtAprilTag(double yaw);

        void SetSpeed(double value);


    private:
        SparkMax m_turretMotor{TurretConstants::kTurretCanId, SparkLowLevel::MotorType::kBrushless};
        frc::PIDController anglePIDController{0.025, 0, 0};
};