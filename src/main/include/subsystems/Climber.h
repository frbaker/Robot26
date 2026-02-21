#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
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
        void Climb();  // Alias for Run()

        /**
         * Returns true if the climb engage limit switch is triggered.
         * This indicates the robot is at the correct position to engage climb.
         */
        bool IsLimitSwitchTriggered();

    private:
        SparkMax m_climberMotor{ClimberConstants::kClimberCanId, SparkLowLevel::MotorType::kBrushless};
        frc::DigitalInput m_limitSwitch{ClimberConstants::kLimitSwitchChannel};
};