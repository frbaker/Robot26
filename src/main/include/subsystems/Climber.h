#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include "Constants.h"

using namespace rev::spark;

class ClimberSubsystem : public frc2::SubsystemBase {
    public:
        ClimberSubsystem();
        void Periodic() override;
    private:
        SparkMax m_climberMotor{ClimberConstants::kClimberCanId, SparkLowLevel::MotorType::kBrushless};
};