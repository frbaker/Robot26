#pragma once

#include <frc/DigitalOutput.h>
#include <frc/PWM.h>
#include <frc2/command/SubsystemBase.h>

class LEDSubsystem : public frc2::SubsystemBase {
    public:
        LEDSubsystem();

        void Periodic() override;

        void GO(float red, float green, float blue);
    private:
        frc::PWM m_redPin{0};
        frc::PWM m_greenPin{1};
        frc::PWM m_bluePin{2};
        
};