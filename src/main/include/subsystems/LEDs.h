#pragma once

#include <frc/DigitalOutput.h>
#include <frc2/command/SubsystemBase.h>
#include "Constants.h"

class LEDSubsystem : public frc2::SubsystemBase {
    public:
        LEDSubsystem();

        void Periodic() override;

        void TurnOnLEDs(float red, float green, float blue);
    private:
        frc::DigitalOutput m_redPin{LEDConstants::kRedPin};
        frc::DigitalOutput m_greenPin{LEDConstants::kGreenPin};
        frc::DigitalOutput m_bluePin{LEDConstants::kBluePin};
};