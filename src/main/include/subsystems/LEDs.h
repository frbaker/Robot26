#pragma once

#include <frc/DigitalOutput.h>
#include <frc2/command/SubsystemBase.h>

class LEDSubsystem : public frc2::SubsystemBase {
    public:
        LEDSubsystem();

        void Periodic() override;

        void TurnOnLEDs(float red, float green, float blue);
    private:
        frc::DigitalOutput m_redPin{3};
        frc::DigitalOutput m_greenPin{4};
        frc::DigitalOutput m_bluePin{5};
};