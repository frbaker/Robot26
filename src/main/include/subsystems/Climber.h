#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include "Constants.h"

using namespace rev::spark;

class ClimberSubsystem : public frc2::SubsystemBase {
    public:
        ClimberSubsystem();

        void Periodic() override;

        void Run();
        void Reverse();
        void Stop();

    private:
        SparkMax m_climberMotor{ClimberConstants::kClimberCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_climberEncoder = m_climberMotor.GetEncoder();
};