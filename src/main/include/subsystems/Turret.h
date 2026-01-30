#pragma once

#include <frc2/command/SubsystemBase.h>
#include <Constants.h>
#include <rev/SparkMax.h>

using namespace rev::spark;


class TurretSubsystem : public frc2::SubsystemBase {
    public:
        TurretSubsystem();


    private:
        SparkMax m_turretMotor{TurretConstants::kTurretCanId, SparkLowLevel::MotorType::kBrushless};
};