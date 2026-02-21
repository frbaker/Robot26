#pragma once

#include <frc2/command/SubsystemBase.h>
#include <frc/DigitalInput.h>
#include <rev/SparkMax.h>
#include "Constants.h"
#include <rev/config/SparkMaxConfig.h>

using namespace rev::spark;

class ClimberSubsystem : public frc2::SubsystemBase {
    public:
        ClimberSubsystem();

        void Periodic() override;

        void Run();
        void Reverse();
        void ReverseBypass();
        void Stop();
        void Climb();  // Alias for Run()

        /**
         * Returns true if the climb engage limit switch is triggered.
         * This indicates the robot is at the correct position to engage climb.
         */
        bool IsLimitSwitchTriggered();

    private:
        SparkMax m_climberMotor{ClimberConstants::kClimberCanId, SparkLowLevel::MotorType::kBrushless};
        SparkRelativeEncoder m_climberEncoder = m_climberMotor.GetEncoder();
        SparkClosedLoopController m_climberController = m_climberMotor.GetClosedLoopController();
        SparkMaxConfig m_climberConfig;
        frc::DigitalInput m_limitSwitch{ClimberConstants::kLimitSwitchChannel};
};
