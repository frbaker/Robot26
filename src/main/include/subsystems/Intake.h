#pragma once

#include <frc2/command/SubsystemBase.h>
#include <rev/SparkMax.h>
#include <rev/config/SparkMaxConfig.h>
#include "Constants.h"

using namespace rev::spark;

class IntakeSubsystem : public frc2::SubsystemBase{
    public:
        IntakeSubsystem();
        void Periodic() override;

        void Run();
        void Reverse();
        void Stop();

        void RaiseLifter();
        void LowerLifter();

        double GetLifterEncoderValue();

        void SetLifter(double value);
    private:
        SparkMax m_intakeMotor{IntakeConstants::kIntakeCanId, SparkLowLevel::MotorType::kBrushless};
        SparkMax m_lifterMotor{IntakeConstants::kLifterCanId, SparkLowLevel::MotorType::kBrushless};

        SparkRelativeEncoder m_lifterEncoder = m_lifterMotor.GetEncoder();
        SparkClosedLoopController m_lifterController = m_lifterMotor.GetClosedLoopController();
        SparkMaxConfig m_lifterConfig;

        SparkMaxConfig m_intakeConfig;
        SparkRelativeEncoder m_intakeEncoder = m_intakeMotor.GetEncoder();
        SparkClosedLoopController m_intakeController = m_intakeMotor.GetClosedLoopController();

        bool m_lifterInBrakeMode = true;
};