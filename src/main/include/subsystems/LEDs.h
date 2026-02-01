#pragma once

#include <frc/DigitalOutput.h>
#include <frc2/command/SubsystemBase.h>

class LEDSubsystem : public frc2::SubsystemBase {
    public:
        LEDSubsystem();

        void Periodic() override;

        void GO(bool red, bool green, bool blue);

    private:
        frc::DigitalOutput m_redPin{0};
        frc::DigitalOutput m_greenPin{1};
        frc::DigitalOutput m_bluePin{2};
};