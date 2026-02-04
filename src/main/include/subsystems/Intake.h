#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include <Constants.h>

using namespace rev::spark;

class IntakeSubsystem : public frc2::SubsystemBase {
    public:
        IntakeSubsystem();

        void Periodic() override;

        void Run();      // Run intake to collect fuel
        void Reverse();  // Reverse to eject
        void Stop();

    private:
        SparkMax m_intakeMotor{IntakeConstants::kIntakeCanId, SparkLowLevel::MotorType::kBrushless};
        SparkMaxConfig m_intakeConfig;
};
